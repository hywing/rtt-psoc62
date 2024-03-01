#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"

#define LED0_PIN     GET_PIN(0, 0)
#define USER_KEY GET_PIN(6, 2)

void irq_callback()
{
    int val = rt_pin_read(USER_KEY);
    if(val == 1) {
        rt_pin_write(LED0_PIN, PIN_HIGH);
    }
    else {
        rt_pin_write(LED0_PIN, PIN_LOW);
    }
}

int main(void)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(USER_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(USER_KEY, PIN_IRQ_MODE_RISING_FALLING, irq_callback, RT_NULL);
    rt_pin_irq_enable(USER_KEY, PIN_IRQ_ENABLE);
    return 0;
}
