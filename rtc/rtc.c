#include <rtthread.h>
#include <rtdevice.h>
#include <time.h>
#include "drv_gpio.h"
#include "drivers/alarm.h"
#include <rtdbg.h>
#include <u8g2_port.h>

#define OLED_I2C_PIN_SCL                    64  // P8.0
#define OLED_I2C_PIN_SDA                    65  // P8.1

#define RTC_NAME "rtc"

static rt_device_t device = RT_NULL;

static int uesr_rtc_init(void)
{
    rt_err_t ret = RT_EOK;
    time_t now;

    device = rt_device_find(RTC_NAME);
    if (!device)
    {
        rt_kprintf("find %s failed!", RTC_NAME);
        return RT_ERROR;
    }

    if(rt_device_open(device, 0) != RT_EOK)
    {
        rt_kprintf("open %s failed!", RTC_NAME);
        return RT_ERROR;
    }

    ret = set_date(2024, 1, 20);
    if (ret != RT_EOK)
    {
        rt_kprintf("set RTC date failed\n");
        return ret;
    }

    ret = set_time(16, 55, 50);
    if (ret != RT_EOK)
    {
        rt_kprintf("set RTC time failed\n");
        return ret;
    }

    now = time(RT_NULL);
    rt_kprintf("RTC device init success,now time is %s\n", ctime(&now));

    return ret;
}

INIT_APP_EXPORT(uesr_rtc_init);

void oled_display()
{
    time_t now;
    struct tm *p_tm;
    u8g2_t u8g2;
    char buffer[50];

    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    // Draw Graphics
    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 1, 18, "U8g2 on RT-Thread");
    u8g2_SendBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
    u8g2_SendBuffer(&u8g2);

    for (;;)
    {
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
        now = time(RT_NULL);
        p_tm = localtime(&now);
        memset(buffer, 0, 50);
        snprintf(buffer, 50, "%d-%d-%d %d:%d:%d\n", p_tm->tm_year+ 1900, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
        u8g2_DrawStr(&u8g2, 1, 18, buffer);
        u8g2_SendBuffer(&u8g2);
        rt_thread_mdelay(1000);
    }
}

int main(void)
{
    oled_display();
    return 0;
}
