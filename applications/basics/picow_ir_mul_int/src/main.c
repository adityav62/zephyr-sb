#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define IR_SENSOR_NODE1 DT_PATH(ir_sensors, ir_sensor_1_input)
#define IR_SENSOR_NODE2 DT_PATH(ir_sensors, ir_sensor_2_input)
#define IR_SENSOR_NODE3 DT_PATH(ir_sensors, ir_sensor_3_input)

//This structure encapsulates all relevant information for each sensor:
struct sensor_data {
    struct gpio_callback cb;
    const struct device *dev;
    struct gpio_dt_spec sensor;
    const char *sensor_name;
    int status;  // Store the status of each sensor
    int last_status; // Store the last known status
};

void clear_screen(const struct device *uart_dev)
{
    char clear_cmd[] = "\033[2J\033[H"; // Clear screen and move cursor to home position
    uart_fifo_fill(uart_dev, (const uint8_t *)clear_cmd, sizeof(clear_cmd) - 1);
}

void ir_sensor_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /*
    #define CONTAINER_OF(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
    
    CONTAINER_OF allows the ISR to access the entire 
    sensor_data structure from just the gpio_callback pointer,
    ensuring that each sensor's context is handled individually
    and avoiding global variables. - essentially calculates
    struct base address using a struct member
    */
    struct sensor_data *data = CONTAINER_OF(cb, struct sensor_data, cb);
    data->status = gpio_pin_get(data->sensor.port, data->sensor.pin);
    
    if (data->status != data->last_status) {
        char buffer[128];
        int len = 0;

        if (data->status == 0) {
            len = snprintf(buffer, sizeof(buffer), "%s: Obstacle detected\r\n", data->sensor_name);
            uart_fifo_fill(data->dev, (const uint8_t *)buffer, len);
        } else {
            clear_screen(data->dev);
        }

        data->last_status = data->status;  // Update the last known status
    }
}

int main(void)
{
    int ret;

    // Define and initialize sensor data for each sensor
    struct sensor_data sensors[] = {
        { .sensor = GPIO_DT_SPEC_GET(IR_SENSOR_NODE1, gpios), .sensor_name = "IR_1" },
        { .sensor = GPIO_DT_SPEC_GET(IR_SENSOR_NODE2, gpios), .sensor_name = "IR_2" },
        { .sensor = GPIO_DT_SPEC_GET(IR_SENSOR_NODE3, gpios), .sensor_name = "IR_3" },
    };
    
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    for (int i = 0; i < ARRAY_SIZE(sensors); i++) {
        sensors[i].dev = dev; //the same ACM device is used to send messages for all sensors over the console
        sensors[i].status = 1;  // Initialize status - 1 indicates No Obstacle since the default behaviour is GPIO_ACTIVE_HIGH
        sensors[i].last_status = 1;  // Initialize last status
        if (!gpio_is_ready_dt(&sensors[i].sensor)) {
            printk("Error: %s GPIO device is not ready\r\n", sensors[i].sensor_name);
            return 0;
        }

        ret = gpio_pin_configure_dt(&sensors[i].sensor, GPIO_INPUT);
        if (ret != 0) {
            printk("Error %d: Failed to configure %s pin\r\n", ret, sensors[i].sensor_name);
            return 0;
        }

        // Register callbacks
        gpio_init_callback(&sensors[i].cb, ir_sensor_isr, BIT(sensors[i].sensor.pin));
        ret = gpio_add_callback(sensors[i].sensor.port, &sensors[i].cb);
        if(ret != 0) {
            printk("Error %d: Failed to add GPIO callback for sensors\r\n", ret);
            return 0;
        }
        ret = gpio_pin_interrupt_configure_dt(&sensors[i].sensor, GPIO_INT_EDGE_BOTH);
        if (ret != 0) {
            printk("Error %d: Failed to configure interrupt on %s pin\r\n", ret, sensors[i].sensor_name);
            return 0;
        }
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

    // Sleep indefinitely, ISR handles sensor events
    while (1) {
        k_sleep(K_SECONDS(1)); 
    }

    return 0;
}
