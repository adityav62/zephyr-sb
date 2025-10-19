#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define SENSOR_NODE DT_NODELABEL(ir_sensor_input_label)
// #define SENSOR_NODE DT_ALIAS(sensor0)

int main(void)
{
    const struct gpio_dt_spec sensor = GPIO_DT_SPEC_GET(SENSOR_NODE, gpios);
    //const struct device *dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart); //used to identify a device using a specific compatible string, not recommended
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    int ret;

    if (!gpio_is_ready_dt(&sensor)) {
        printk("Error: Sensor GPIO device is not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&sensor, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure sensor pin\n", ret);
        return 0;
    }

    if (!device_is_ready(dev)) {
        printk("CDC ACM device not ready\n");
        return 0;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\n");
        return 0;
    }

    /* Wait for the USB connection */
    k_sleep(K_SECONDS(1));

    while (1) {
        int val = gpio_pin_get_dt(&sensor);
        char buffer[64];
        
        /*`snprintf` is used to format a string and store it in a buffer. 
        It is part of the Standard Library and offers a safer way
        to format strings compared to sprintf, as it prevents buffer
        overflows by limiting the number of characters written.*/

        //format of snprintf --> int snprintf(char *str, size_t size, const char *format, ...)

        int buffer_len = snprintf(buffer, sizeof(buffer), "Sensor value: %d\r\n", val);

        /*uart_fifo_fill is a function that writes data 
        to the UART hardware FIFO buffer. It takes three 
        arguments: the UART device, a pointer to the data 
        buffer, and the number of bytes to write. The 
        function attempts to transmit the data over UART 
        and returns the number of bytes actually written to 
        the buffer, which can be less than the requested length 
        if the FIFO is full. This allows for efficient, 
        non-blocking data transmission over UART.*/
        uart_fifo_fill(dev, buffer, buffer_len);
        
        //Can be used for logging and debugging, maybe to identify which sensor is causing trouble in an array of sensors 
        //printk("%s", buffer);

        k_sleep(K_MSEC(400));
    }
    return 0;
}
