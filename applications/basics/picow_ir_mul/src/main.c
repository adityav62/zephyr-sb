#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define IR_SENSOR_NODE1 DT_PATH(ir_sensors, ir_sensor_1_input)
#define IR_SENSOR_NODE2 DT_PATH(ir_sensors, ir_sensor_2_input)
#define IR_SENSOR_NODE3 DT_PATH(ir_sensors, ir_sensor_3_input)

int main(void)
{
    const struct gpio_dt_spec sensor1 = GPIO_DT_SPEC_GET(IR_SENSOR_NODE1, gpios);
    const struct gpio_dt_spec sensor2 = GPIO_DT_SPEC_GET(IR_SENSOR_NODE2, gpios);
    const struct gpio_dt_spec sensor3 = GPIO_DT_SPEC_GET(IR_SENSOR_NODE3, gpios);
    
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    int ret;

    if (!gpio_is_ready_dt(&sensor1) || !gpio_is_ready_dt(&sensor2) || !gpio_is_ready_dt(&sensor3)) {
        printk("Error: One or more Sensor GPIO devices are not ready\r\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&sensor1, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure sensor1 pin\r\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&sensor2, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure sensor2 pin\r\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&sensor3, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure sensor3 pin\r\n", ret);
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
        int val1 = gpio_pin_get_dt(&sensor1);
        int val2 = gpio_pin_get_dt(&sensor2);
        int val3 = gpio_pin_get_dt(&sensor3);
        
        char buffer[128];

        int buffer_len = snprintf(buffer, sizeof(buffer), "IR_1: %d\t, IR_2: %d\t, IR_3: %d\r\n", val1, val2, val3);

        uart_fifo_fill(dev, buffer, buffer_len);
        
        //Can be used for logging and debugging, maybe to identify which sensor is causing trouble in an array of sensors 
        //printk("%s", buffer);
        k_sleep(K_MSEC(400));
    }
    return 0;
}