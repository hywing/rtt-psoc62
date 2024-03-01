#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"

#define LED_PIN0     GET_PIN(0, 0)
#define LED_PIN1     GET_PIN(0, 1)

void func()
{
    int delay = 500;
    rt_pin_mode(LED_PIN0, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN1, PIN_MODE_OUTPUT);

    for (;;)
    {
        rt_pin_write(LED_PIN0, PIN_HIGH);
        rt_pin_write(LED_PIN1, PIN_HIGH);
        rt_thread_mdelay(delay);
        rt_pin_write(LED_PIN0, PIN_LOW);
        rt_pin_write(LED_PIN1, PIN_LOW);
        rt_thread_mdelay(delay);
    }
}

int main(void)
{
    func();
    return 0;
}
