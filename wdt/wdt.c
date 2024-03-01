#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"

#define WDT_DEV    "wdt"
#define LED_PIN     GET_PIN(0, 1)
#define USER_KEY    GET_PIN(6, 2)

static rt_device_t wdg_dev;

static void idle_hook(void)
{
    rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
}

int init_wdt(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout = 3;

    wdg_dev = rt_device_find(WDT_DEV);
    if (!wdg_dev)
    {
        rt_kprintf("find %s failed!\n", WDT_DEV);
        return RT_ERROR;
    }

     rt_device_init(wdg_dev);

    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    if (ret != RT_EOK)
    {
        rt_kprintf("set %s timeout failed!\n", WDT_DEV);
        return RT_ERROR;
    }

    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("start %s failed!\n", WDT_DEV);
        return -RT_ERROR;
    }

    rt_thread_idle_sethook(idle_hook);

    return ret;
}

// write a bug to crash
void irq_callback()
{
    rt_kprintf("To be crashed!\n");

    char *p = NULL;
    *p = 1080;
}

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(USER_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(USER_KEY, PIN_IRQ_MODE_RISING_FALLING, irq_callback, RT_NULL);
    rt_pin_irq_enable(USER_KEY, PIN_IRQ_ENABLE);

    init_wdt();

    for (;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return 0;
}
