#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define TRIGGER_NODE DT_NODELABEL(trig_pin_label)
#define ECHO_NODE DT_NODELABEL(echo_pin_label)

static const struct gpio_dt_spec trigger = GPIO_DT_SPEC_GET(TRIGGER_NODE, gpios);
static const struct gpio_dt_spec echo = GPIO_DT_SPEC_GET(ECHO_NODE, gpios);

static const struct device *usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

//Function to measure distance
static int measure_distance(void){
    //Set TRIG low for a short duration to ensure a clean signal
    gpio_pin_set_dt(&trigger, 0);
    k_sleep(K_USEC(2));

    //Send a 10 us high pulse to TRIG to start measurement
    gpio_pin_set_dt(&trigger, 1);
    k_sleep(K_USEC(10));
    gpio_pin_set_dt(&trigger, 0);

    //Measure the Pulse width on ECHO
    uint32_t pulse_start = k_cycle_get_32();
    while(gpio_pin_get_dt(&echo) == 0){
        pulse_start = k_cycle_get_32();
    }

    uint32_t pulse_end = k_cycle_get_32();
    while(gpio_pin_get_dt(&echo) == 1){
        pulse_end = k_cycle_get_32();
    }

    // Calculate Duration of High Pulses with overflow handling 
    uint32_t pulse_duration; 
    if (pulse_end >= pulse_start) { 
        pulse_duration = pulse_end - pulse_start; 
    } else { 
        pulse_duration = (UINT32_MAX - pulse_start) + pulse_end + 1; 
    }
    
    //Convert to Microseconds
    uint32_t pulse_duration_us = k_cyc_to_us_near32(pulse_duration);

    //Calculate Distance in centimeters (speed of sound = 34300 cm/s)
    int distance = (pulse_duration_us * 34300) / (2 * 1000000);

    return distance;
}

int main(void)
{ 
    int ret;
    
    if (!device_is_ready(usb_acm)) {
        printk("CDC ACM device not ready\r\n");
        return 0;
    }

    if(!device_is_ready(trigger.port) || !device_is_ready(echo.port)){
        printk("GPIO Device is not ready!\r\n");
        return 0;
    }
    ret = gpio_pin_configure_dt(&trigger, GPIO_OUTPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure trigger pin\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&echo, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure echo pin\n", ret);
        return 0;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\n");
        return 0;
    }

    /* Wait for the USB connection */
    k_sleep(K_SECONDS(1));

    while(1){
        int distance = measure_distance();
        char buffer[64];
       
        //Presents the Output in a cleaner manner by just updating the value
        int len = snprintf(buffer, sizeof(buffer),
                           "\033[2J\033[H"
                           "Ultrasonic Sensor Test\r\n\r\n"
                           "Distance: %d cm\r\n", distance);

        uart_fifo_fill(usb_acm, (uint8_t *) buffer, len);

        k_sleep(K_MSEC(400));
    }

    return 0;
}