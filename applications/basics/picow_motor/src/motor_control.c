#include "motor_control.h"

#define PWM_NODE DT_NODELABEL(pwm_a_label)
#define PWM_PERIOD DT_PWMS_PERIOD(PWM_NODE) //NOTE: converts period to ns, so 20 ms = 20,000,000 ns

// GPIO and PWM specifications
void configure_gpio(const struct gpio_dt_spec *pin, const char *name) {
    int ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Error %d: Failed to configure %s pin\r\n", ret, name);
    } else {
        printk("%s pin configured\r\n", name);
    }
}

void set_motor_speed(const struct pwm_dt_spec *pwm, uint32_t pwm_period, uint32_t pwm_pulse) {
    if (pwm_set_dt(pwm, pwm_period, pwm_pulse) != 0) {
        printk("Error: Failed to set PWM signal\r\n");
    }
}

void ramp_motor_speed_both(uint32_t start_pulse, uint32_t end_pulse, uint32_t ramp_time) {
    int steps = 10;
    int step_delay = ramp_time / steps;
    int32_t step_pulse;
    
    if (end_pulse == 0) {
        step_pulse = - (int32_t) start_pulse / steps; //typecast necessary when converting from uint32_t to int32_t
    } else {
        step_pulse = (end_pulse - start_pulse) / steps;
    }

    for (int i = 0; i <= steps; ++i) {
        int32_t pulse = start_pulse + step_pulse * i;
        set_motor_speed(&pwm_a, PWM_PERIOD, pulse);
        set_motor_speed(&pwm_b, PWM_PERIOD, pulse);
        k_sleep(K_MSEC(step_delay));
    }   
}

void run_motors_forward(uint32_t pwm_pulse) {
    ramp_motor_speed_both(0, pwm_pulse, 500); // Smooth transition
    gpio_pin_set_dt(&ain_1, 1);
    gpio_pin_set_dt(&ain_2, 0);
    gpio_pin_set_dt(&bin_1, 1);
    gpio_pin_set_dt(&bin_2, 0);
}

void run_motors_backward(uint32_t pwm_pulse) {
    ramp_motor_speed_both(0, pwm_pulse, 500); // Smooth transition
    gpio_pin_set_dt(&ain_1, 0);
    gpio_pin_set_dt(&ain_2, 1);
    gpio_pin_set_dt(&bin_1, 0);
    gpio_pin_set_dt(&bin_2, 1);
}

void stop_motors(uint32_t current_pulse) {
    ramp_motor_speed_both(current_pulse, 0, 500); // Smooth transition to stop
    gpio_pin_set_dt(&ain_1, 0);
    gpio_pin_set_dt(&ain_2, 0);
    gpio_pin_set_dt(&bin_1, 0);
    gpio_pin_set_dt(&bin_2, 0);
}
