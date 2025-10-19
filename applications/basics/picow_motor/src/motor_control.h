#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

static const struct gpio_dt_spec ain_1 = GPIO_DT_SPEC_GET(DT_NODELABEL(ain1_pin_label), gpios);
static const struct gpio_dt_spec ain_2 = GPIO_DT_SPEC_GET(DT_NODELABEL(ain2_pin_label), gpios);
static const struct gpio_dt_spec bin_1 = GPIO_DT_SPEC_GET(DT_NODELABEL(bin1_pin_label), gpios);
static const struct gpio_dt_spec bin_2 = GPIO_DT_SPEC_GET(DT_NODELABEL(bin2_pin_label), gpios);
 
static const struct pwm_dt_spec pwm_a = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_a_label));
static const struct pwm_dt_spec pwm_b = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_b_label));

// Function prototypes
void configure_gpio(const struct gpio_dt_spec *pin, const char *name);
void set_motor_speed(const struct pwm_dt_spec *pwm, uint32_t pwm_period, uint32_t pwm_pulse);
void ramp_motor_speed_both(uint32_t start_pulse, uint32_t end_pulse, uint32_t ramp_time);
void run_motors_forward(uint32_t pwm_pulse);
void run_motors_backward(uint32_t pwm_pulse);
void stop_motors(uint32_t current_pulse);
#endif // MOTOR_CONTROL_H
