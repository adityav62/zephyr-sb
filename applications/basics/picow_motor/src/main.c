#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include "motor_control.h"

static const struct device *usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

int main(void) {
    int ret;

    if (!device_is_ready(usb_acm)) {
        printk("CDC ACM device not ready\r\n");
        return 0;
    }

    // Configure GPIO pins for motor control
    configure_gpio(&ain_1, "AIN1");
    configure_gpio(&ain_2, "AIN2");
    configure_gpio(&bin_1, "BIN1");
    configure_gpio(&bin_2, "BIN2");

    // Enable USB device
    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\r\n");
        return 0;
    }

    // Wait for the USB connection
    uint32_t dtr = 0;
    while (!dtr) {
        uart_line_ctrl_get(usb_acm, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    // Configure PWM for the motor driver
    if (!pwm_is_ready_dt(&pwm_a) || !pwm_is_ready_dt(&pwm_b)) {
        printk("PWM Device not ready!\r\n");
        return 0;
    }

   while (1) {
    // Speed variations
    uint32_t pwm_pulse_speeds[] = {PWM_MSEC(5), PWM_MSEC(10), PWM_MSEC(15), PWM_MSEC(20)};
    
    for (int i = 0; i < 4; ++i) {
        printk("Ramping both motors forward to speed %d\n", i + 1);
        run_motors_forward(pwm_pulse_speeds[i]);
        k_sleep(K_MSEC(1000)); // Run for 1 second

        printk("Stopping motors\n");
        stop_motors(pwm_pulse_speeds[i]);
        k_sleep(K_MSEC(500)); // Pause for 500ms before reversing direction

        printk("Ramping both motors backward to speed %d\n", i + 1);
        run_motors_backward(pwm_pulse_speeds[i]);
        k_sleep(K_MSEC(1000)); // Run for 1 second

        printk("Stopping motors\n");
        stop_motors(pwm_pulse_speeds[i]);
        k_sleep(K_MSEC(500)); // Pause for 500ms before next iteration
        }
    }
    return 0;
}
