#include <rtthread.h>
#include <rtdbg.h>
#include <board.h>
#include <rtdevice.h>
#include <u8g2_port.h>

#define OLED_I2C_PIN_SCL                    64  // P8.0
#define OLED_I2C_PIN_SDA                    65  // P8.1

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL     0           /* ADC 通道 */

int main(void)
{
    // OLED
    u8g2_t u8g2;
    char buffer[50];
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    // ADC1
    rt_adc_device_t adc_dev;
    rt_uint32_t value;
    rt_err_t ret = RT_EOK;

    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
    }
    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
    if(ret == RT_EOK)
    {
        rt_kprintf("adc sample run success!  find %s device!\n", ADC_DEV_NAME);
    }

    while(1)
    {
        // read adc value
        value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
        memset(buffer, 0, 50);

        if(4294965 <= value) value = 0;
        snprintf(buffer, 50, "Voltage : %d.%02dV\n", value / 1000, value % 1000);

        // display on OLED
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);

        u8g2_DrawStr(&u8g2, 20, 36, buffer);
        u8g2_SendBuffer(&u8g2);
        cyhal_system_delay_ms(200);
    }
    return 0;
}
