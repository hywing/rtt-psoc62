#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"

#define DS18B20_PIN         GET_PIN(10, 5)
#define DS18B20_DQ_OUT(x)   rt_pin_write(DS18B20_PIN, x)
#define DS18B20_DQ_IN       rt_pin_read(DS18B20_PIN)

// ��λDS18B20
static void ds18b20_reset(void)
{
    DS18B20_DQ_OUT(0);  /* ����DQ,��λ */
    rt_hw_us_delay(750);      /* ����750us */
    DS18B20_DQ_OUT(1);  /* DQ=1, �ͷŸ�λ */
    rt_hw_us_delay(15);       /* �ӳ�15US */
}

// �ȴ�DS18B20�Ļ�Ӧ
uint8_t ds18b20_check(void)
{
    uint8_t retry = 0;
    uint8_t rval = 0;

    while (DS18B20_DQ_IN && retry < 200)    /* �ȴ�DQ���, �ȴ�200us */
    {
        retry++;
        rt_hw_us_delay(1);
    }

    if (retry >= 200)
    {
        rval = 1;
    }
    else
    {
        retry = 0;

        while (!DS18B20_DQ_IN && retry < 240)   /* �ȴ�DQ���, �ȴ�240us */
        {
            retry++;
            rt_hw_us_delay(1);
        }

        if (retry >= 240) rval = 1;
    }

    return rval;
}

// ��DS18B20��ȡһ��λ
static uint8_t ds18b20_read_bit(void)
{
    uint8_t data = 0;
    DS18B20_DQ_OUT(0);
    rt_hw_us_delay(2);
    DS18B20_DQ_OUT(1);
    rt_hw_us_delay(12);

    if (DS18B20_DQ_IN)
    {
        data = 1;
    }

    rt_hw_us_delay(50);
    return data;
}

// ��DS18B20��ȡһ���ֽ�
static uint8_t ds18b20_read_byte(void)
{
    uint8_t i, b, data = 0;

    for (i = 0; i < 8; i++)
    {
        b = ds18b20_read_bit(); /* DS18B20�������λ���� ,��λ���ݺ���� */

        data |= b << i;         /* ���data��ÿһλ */
    }

    return data;
}

// дһ���ֽڵ�DS18B20
static void ds18b20_write_byte(uint8_t data)
{
    uint8_t j;

    for (j = 1; j <= 8; j++)
    {
        if (data & 0x01)
        {
            DS18B20_DQ_OUT(0);  /*  Write 1 */
            rt_hw_us_delay(2);
            DS18B20_DQ_OUT(1);
            rt_hw_us_delay(60);
        }
        else
        {
            DS18B20_DQ_OUT(0);  /*  Write 0 */
            rt_hw_us_delay(60);
            DS18B20_DQ_OUT(1);
            rt_hw_us_delay(2);
        }

        data >>= 1;             /* ����,��ȡ��һλ���� */
    }
}

// ��ʼ�¶�ת��
static void ds18b20_start(void)
{
    ds18b20_reset();
    ds18b20_check();
    ds18b20_write_byte(0xcc);   /*  skip rom */
    ds18b20_write_byte(0x44);   /*  convert */
}

// ��ʼ��DS18B20��IO�� DQ ͬʱ���DS18B20�Ĵ���
uint8_t ds18b20_init(void)
{
    rt_pin_mode(DS18B20_PIN, PIN_MODE_OUTPUT_OD);
//    rt_pin_write(DS18B20_PIN, PIN_HIGH);

    ds18b20_reset();
    return ds18b20_check();
}

// ��ds18b20�õ��¶�ֵ(���ȣ�0.1C)
short ds18b20_get_temperature(void)
{
    uint8_t flag = 1;           /* Ĭ���¶�Ϊ���� */
    uint8_t TL, TH;
    short temp;

    ds18b20_start();            /*  ds1820 start convert */
    ds18b20_reset();
    ds18b20_check();
    ds18b20_write_byte(0xcc);   /*  skip rom */
    ds18b20_write_byte(0xbe);   /*  convert */
    TL = ds18b20_read_byte();   /*  LSB */
    TH = ds18b20_read_byte();   /*  MSB */

    if (TH > 7)
    {
        TH = ~TH;
        TL = ~TL;
        flag = 0;   /* �¶�Ϊ�� */
    }

    temp = TH;      /* ��ø߰�λ */
    temp <<= 8;
    temp += TL;     /* ��õװ�λ */
    temp = (double)temp * 0.625;    /* ת�� */

    if (flag == 0)
    {
        temp = -temp;   /* ���¶�ת���ɸ��¶� */
    }

    return temp;
}

///////////////////////////////////////////////////

#include "stdlib.h"
#include "oledfont.h"

// ��PSoc62���ӵ�GPIO
#define CS_PIN      GET_PIN(10, 0)
#define DC_PIN      GET_PIN(10, 1)
#define RES_PIN     GET_PIN(10, 2)
#define SDA_PIN     GET_PIN(10, 3)
#define SCL_PIN     GET_PIN(10, 4)
#define WR_PIN      GET_PIN(0, 1)
#define RD_PIN      GET_PIN(0, 1)

// OLEDģʽ����
#define OLED_MODE   0
#define SIZE        16
#define XLevelL     0x00
#define XLevelH     0x10
#define Max_Column  128
#define Max_Row     64
#define Brightness  0xFF
#define X_WIDTH     128
#define Y_WIDTH     64

// OLED�˿ڶ���
#define OLED_CS_Clr()   rt_pin_write(CS_PIN, PIN_LOW)
#define OLED_CS_Set()   rt_pin_write(CS_PIN, PIN_HIGH)
#define OLED_RST_Clr()  rt_pin_write(RES_PIN, PIN_LOW)
#define OLED_RST_Set()  rt_pin_write(RES_PIN, PIN_HIGH)
#define OLED_DC_Clr()   rt_pin_write(DC_PIN, PIN_LOW)
#define OLED_DC_Set()   rt_pin_write(DC_PIN, PIN_HIGH)
#define OLED_SDIN_Clr() rt_pin_write(SDA_PIN, PIN_LOW)
#define OLED_SDIN_Set() rt_pin_write(SDA_PIN, PIN_HIGH)
#define OLED_SCLK_Clr() rt_pin_write(SCL_PIN, PIN_LOW)
#define OLED_SCLK_Set() rt_pin_write(SCL_PIN, PIN_HIGH)
#define OLED_WR_Clr()   rt_pin_write(WR_PIN, PIN_LOW)
#define OLED_WR_Set()   rt_pin_write(WR_PIN, PIN_HIGH)
#define OLED_RD_Clr()   rt_pin_write(RD_PIN, PIN_LOW)
#define OLED_RD_Set()   rt_pin_write(RD_PIN, PIN_HIGH)

#define OLED_CMD  0 // д����
#define OLED_DATA 1 // д����

#define u8 unsigned char
#define u32 unsigned int

static u8 buff[20];

// ��SSD1309д��һ���ֽ����ݣ�dat -> Ҫд������� or���cmd -> ����or����
void OLED_WR_Byte(u8 dat, u8 cmd)
{
    u8 i;
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    for(i=0; i < 8; i++)
    {
        OLED_SCLK_Clr();
        if(dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat <<= 1;
    }
    OLED_CS_Set();
    OLED_DC_Set();
}

// ���û��Ƶ�����
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f) | 0x01, OLED_CMD);
}

// ����OLED��ʾ
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X14, OLED_CMD);
    OLED_WR_Byte(0XAF, OLED_CMD);
}

// �ر�OLED��ʾ
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X10, OLED_CMD);
    OLED_WR_Byte(0XAE, OLED_CMD);
}

// ������������Ļ����Ϊ��ɫ
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0;i<8;i++)
    {
        OLED_WR_Byte (0xb0 + i, OLED_CMD);      // ����ҳ��ַ��0~7��
        OLED_WR_Byte (0x00, OLED_CMD);          // ������ʾλ�á��е͵�ַ
        OLED_WR_Byte (0x10, OLED_CMD);          // ������ʾλ�á��иߵ�ַ
        for(n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    }
}

// �����ַ�
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1)
    {
        x=0;
        y=y+2;
    }
    if(SIZE ==16)
    {
        OLED_Set_Pos(x,y);
        for(i=0;i<8;i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0;i<8;i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
    }
    else {
        OLED_Set_Pos(x,y+1);
        for(i=0;i<6;i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);

    }
}

// ��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        OLED_ShowChar(x,y,chr[j]);
        x+=8;
        if(x>120)
        {
            x=0;y+=2;
        }
        j++;
    }
}

// ��ʾ����
void OLED_ShowNum(u8 x,u8 y,u32 num)
{
    rt_memset(buff, 20, 0);
    rt_sprintf(buff, "%d", num);
    OLED_ShowString(x, y, buff);
}

// ��ʾ����
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{
    u8 t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0;t<16;t++)
    {
        OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
        adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
    {
        OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
        adder+=1;
    }
}


// ��ʼ��SSD1309
void OLED_Init(void)
{
    // init gpios
    rt_pin_mode(CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(DC_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RES_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SDA_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SCL_PIN, PIN_MODE_OUTPUT);

    OLED_RST_Set();
    rt_thread_mdelay(100);
    OLED_RST_Clr();
    rt_thread_mdelay(100);
    OLED_RST_Set();

    OLED_WR_Byte(0xAE,OLED_CMD);    //--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);    //---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);    //---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);    //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);    //--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD);    // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);    //--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
    OLED_WR_Byte(0xC8,OLED_CMD);    //Set COM/Row Scan Direction   0xc0���·��� 0xc8����
    OLED_WR_Byte(0xA6,OLED_CMD);    //--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);    //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);    //--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);    //-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);    //-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);    //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);    //--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);    //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);    //--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);    //--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);    //Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);    //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);    //
    OLED_WR_Byte(0x8D,OLED_CMD);    //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);    //--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);    // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);    // Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);    //--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD);    /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);
}

//////////////////////////
#define BH1750_I2C_BUS_NAME     "i2c1"
#define BH1750_ADDR             0x23

static rt_uint8_t buffer[2];
static uint16_t light = 0;
static struct rt_i2c_bus_device *i2c_bus;

static int read_i2c_reg(rt_uint8_t reg_addr, rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    msgs[0].addr = BH1750_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1;

    msgs[1].addr = BH1750_ADDR;
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

static int8_t write_i2c_reg(uint8_t data)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = BH1750_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = NULL;
    msgs[0].len   = 0;

    msgs[1].addr  = BH1750_ADDR;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf   = &data;
    msgs[1].len   = 1;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

void BH1750_Init()
{
    write_i2c_reg(0x10);
}

int main(void)
{
    // ds18b20
    ds18b20_init();

    // bh1750
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(BH1750_I2C_BUS_NAME);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", BH1750_I2C_BUS_NAME);
        return RT_ERROR;
    }

    BH1750_Init();
    rt_thread_mdelay(500);

    // ssd1309
    OLED_Init();
    OLED_Clear();

    for (;;)
    {
        // ds18b20
        int val = ds18b20_get_temperature();
        rt_memset(buff, 20, 0);
        rt_sprintf(buff, "temp : %d.%d'C  ", val / 10, val % 10);
        OLED_ShowString(10, 2, buff);
        rt_thread_mdelay(200);

        // bh1750
        BH1750_Init();
        rt_thread_mdelay(200);
        read_i2c_reg(0, buffer, 2);
        light = ((buffer[0] << 8) | buffer[1]) / 1.2;
        rt_memset(buff, 20, 0);
        rt_sprintf(buff, "light : %d lx      ", light);
        OLED_ShowString(10, 5, buff);
        rt_thread_mdelay(150);
    }
}