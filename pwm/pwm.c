#include <rtthread.h>
#include <rtdevice.h>

#include <rtthread.h>
#include "board.h"
#include <rtdevice.h>

#define PWM_LED_DEV     "pwm0"
#define PWM_CHANNEL     0

int main(void)
{
    struct rt_device_pwm *pwm_dev = RT_NULL;
    rt_uint32_t period, pulse, up;

    int gap = 2000;
    int sec = 5;

    period = 500000;
    up = 1;
    pulse = 0;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_LED_DEV);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm device (%s) not found!\n", PWM_LED_DEV);
        return RT_ERROR;
    }

    rt_pwm_set(pwm_dev, PWM_CHANNEL, period, pulse);
    rt_pwm_enable(pwm_dev, PWM_CHANNEL);

    while (1)
    {
        rt_thread_mdelay(sec);
        if (up)
        {
            pulse += gap;
        }
        else
        {
            pulse -= gap;
        }

        if (pulse >= period)
        {
            up = 0;
        }
        if (0 >= pulse)
        {
            up = 1;
        }

        rt_pwm_set(pwm_dev, PWM_CHANNEL, period, pulse);
    }
}
