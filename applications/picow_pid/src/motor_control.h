/* motor_control.h */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

// Initialize motor control hardware
int motor_init(void);

// Set motor speed with direction control
// This function will be called from the PID controller
void motor_set_speed(float speed);

#endif /* MOTOR_CONTROL_H */