#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"

#define LED_PIN     GET_PIN(0, 1)
#define UART_DEVICE_NAME       "uart0"
#define BSP_UART4_RX_BUFSIZE 1024

static rt_device_t serial;

struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

static struct rt_messagequeue rx_mq;

static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_pin_write(LED_PIN, PIN_LOW);
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        rt_kprintf("message queue full£¡\n");
    }
    return result;
}

static void serial_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    static char rx_buffer[BSP_UART4_RX_BUFSIZE + 1];
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
//        if (result == RT_EOK)
        {
            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
            rx_buffer[rx_length] = '\0';
            rt_device_write(serial, 0, rx_buffer, rx_length);
            rt_pin_write(LED_PIN, PIN_HIGH);
            rt_kprintf("%s\n",rx_buffer);
        }
    }
}

static int uart0_setup(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    static char msg_pool[256];
    char str[] = "uart0 setup successfully!\r\n";

    if (argc == 2)
    {
        rt_strncpy(uart_name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(uart_name, UART_DEVICE_NAME, RT_NAME_MAX);
    }

    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_9600;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.bufsz     = 1024;
    config.parity    = PARITY_NONE;
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    rt_mq_init(&rx_mq, "rx_mq", msg_pool, sizeof(struct rx_msg), sizeof(msg_pool), RT_IPC_FLAG_FIFO);
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(serial, uart_input);
    rt_device_write(serial, 0, str, (sizeof(str) - 1));
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}

MSH_CMD_EXPORT(uart0_setup, uart0 setup example);


int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_thread_mdelay(500);
    }
}
