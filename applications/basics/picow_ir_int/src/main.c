#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define IR_NODE DT_NODELABEL(ir_sensor_input_label)

//static const struct gpio_dt_spec ir_sensor = GPIO_DT_SPEC_GET(IR_NODE, gpios);
//static const struct device *acm_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
//static struct gpio_callback ir_sensor_cb_data;

struct sensor_data{
    struct gpio_callback cb;
    const struct device *acm_dev;
    struct gpio_dt_spec ir_sensor;
};

void ir_sensor_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct sensor_data *data = CONTAINER_OF(cb, struct sensor_data, cb); 
    int val = gpio_pin_get(data->ir_sensor.port, data->ir_sensor.pin);
    char buffer[64];
    int buffer_len = snprintf(buffer, sizeof(buffer), "%s\r\n", (val == 0) ? "Obstacle detected!" : "No Obstacle..."); 
    
    // Send the message over UART 
    uart_fifo_fill(data->acm_dev, (const uint8_t *)buffer, buffer_len);
}

int main(void)
{
    int ret;

    struct sensor_data ir_sensor_data = {
        .ir_sensor =  GPIO_DT_SPEC_GET(IR_NODE, gpios),
        .acm_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
    };

    if (!gpio_is_ready_dt(&ir_sensor_data.ir_sensor)) {
        printk("Error: Sensor GPIO device is not ready\r\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&ir_sensor_data.ir_sensor, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure sensor pin\r\n", ret);
        return 0;
    }

    //------Registering callbacks-----
    gpio_init_callback(&ir_sensor_data.cb, ir_sensor_isr, BIT(ir_sensor_data.ir_sensor.pin));
    gpio_add_callback(ir_sensor_data.ir_sensor.port, &ir_sensor_data.cb);
    ret = gpio_pin_interrupt_configure_dt(&ir_sensor_data.ir_sensor, GPIO_INT_EDGE_BOTH);
    if(ret!=0)
    {
        printk("Error %d: Failed to configure interrupt on sensor pin\r\n", ret);
        return 0;
    }
    //------------------------------------------

    if (!device_is_ready(ir_sensor_data.acm_dev)) {
        printk("CDC ACM device not ready\r\n");
        return 0;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\r\n");
        return 0;
    }

    /* Wait for the USB connection */
    k_sleep(K_SECONDS(1));

    while (1) {
        k_sleep(K_FOREVER); //sleep indefinitely, ISR handles sensor events
    }
    return 0;
}