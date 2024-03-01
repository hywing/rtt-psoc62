#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"

#define LED_PIN                 GET_PIN(0, 1)
#define BME280_I2C_BUS_NAME     "i2c1"
#define BME280_ADDR             0x76

static struct rt_i2c_bus_device *i2c_bus;
static rt_thread_t bme280_thread = RT_NULL;
static unsigned long int hum_raw,temp_raw,pres_raw;
static rt_uint8_t data[8];

static signed long int t_fine;
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static int8_t  dig_H1;
static int16_t dig_H2;
static int8_t  dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t  dig_H6;
static signed long int temp_cal;
static unsigned long int press_cal,hum_cal;
static double temp_act;
static double press_act;
static double hum_act;

static signed long int calibration_T(signed long int adc_T)
{

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

static unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);
}

static int read_bme280_reg(rt_uint8_t reg_addr, rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    msgs[0].addr = BME280_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1;

    msgs[1].addr = BME280_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = data;
    msgs[1].len = len;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
        return -RT_ERROR;
}

static int8_t write_bme280_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = BME280_ADDR;                        /* Slave address */
    msgs[0].flags = RT_I2C_WR;                          /* Write flag */
    msgs[0].buf   = &tmp;                               /* Slave register address */
    msgs[0].len   = 1;                                  /* Number of bytes sent */

    msgs[1].addr  = BME280_ADDR;                               /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;                               /* Read data pointer */
    msgs[1].len   = len;                                /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void readCalibrationData()
{
    uint8_t data[32];
    read_bme280_reg(0x88, data, 24);
    read_bme280_reg(0xa1, data + 24, 1);
    read_bme280_reg(0xe1, data + 25, 7);

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];
}

static int init_bme280(void)
{
    i2c_bus = (struct rt_i2c_bus_device *) rt_device_find(BME280_I2C_BUS_NAME);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", BME280_I2C_BUS_NAME);
        return RT_ERROR;
    }

    rt_uint8_t data;
    int size = read_bme280_reg(0xD0, &data, 1);
    rt_kprintf("bme280 device id : %x\n", data);

    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off
    uint8_t spi3w_en = 0;           //3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    write_bme280_reg(0xF2, &ctrl_hum_reg, 1);
    write_bme280_reg(0xF4, &ctrl_meas_reg, 1);
    write_bme280_reg(0xF5, &config_reg, 1);

    readCalibrationData();

    return RT_EOK;
}

static void bme280_entry(void* paremeter)
{
    init_bme280();

    while(1)
    {
        read_bme280_reg(0xf7, data, 8);
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
        hum_raw  = (data[6] << 8) | data[7];

        temp_cal = calibration_T(temp_raw);
        press_cal = calibration_P(pres_raw);
        hum_cal = calibration_H(hum_raw);
        temp_act = (double)temp_cal / 100.0;
        press_act = (double)press_cal;
        hum_act = (double)hum_cal / 1024.0;

        rt_kprintf("Temperature=%4dC, Humidity=%4d%, Pressure=%4dPa \r\n",(int)temp_act,(int)hum_act,(int)press_act);
        rt_thread_mdelay(500);
    }
}


int main(void)
{
    bme280_thread = rt_thread_create("bme280", bme280_entry, RT_NULL, 1024, 16, 20);
    if(bme280_thread != RT_NULL)
    {
        rt_thread_startup(bme280_thread);
    }

    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    for (;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
