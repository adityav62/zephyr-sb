/* motor_control.c */
#include "motor_control.h"

LOG_MODULE_DECLARE(main);

// Define PWM and GPIO nodes from devicetree
#define PWM_NODE_A DT_NODELABEL(pwm_a_label)
#define PWM_NODE_B DT_NODELABEL(pwm_b_label)
#define AIN1_NODE DT_NODELABEL(ain1_pin_label)
#define AIN2_NODE DT_NODELABEL(ain2_pin_label)
#define BIN1_NODE DT_NODELABEL(bin1_pin_label)
#define BIN2_NODE DT_NODELABEL(bin2_pin_label)

#define PWM_PERIOD DT_PWMS_PERIOD(PWM_NODE_A) // Period in ns
#define MAX_PULSE (PWM_PERIOD) // 100% duty cycle as maximum
#define MIN_PULSE (PWM_PERIOD / 10) //10% duty cycle as minimum

// Device specs
static const struct gpio_dt_spec ain_1 = GPIO_DT_SPEC_GET(AIN1_NODE, gpios);
static const struct gpio_dt_spec ain_2 = GPIO_DT_SPEC_GET(AIN2_NODE, gpios);
static const struct gpio_dt_spec bin_1 = GPIO_DT_SPEC_GET(BIN1_NODE, gpios);
static const struct gpio_dt_spec bin_2 = GPIO_DT_SPEC_GET(BIN2_NODE, gpios);
 
static const struct pwm_dt_spec pwm_a = PWM_DT_SPEC_GET(PWM_NODE_A);
static const struct pwm_dt_spec pwm_b = PWM_DT_SPEC_GET(PWM_NODE_B);

int motor_init(void) {
    // Check if devices are ready
    if (!device_is_ready(pwm_a.dev) || !device_is_ready(pwm_b.dev) ||
        !device_is_ready(ain_1.port) || !device_is_ready(ain_2.port) ||
        !device_is_ready(bin_1.port) || !device_is_ready(bin_2.port)) {
        LOG_ERR("Motor devices not ready");
        return -1;
    }

    // Configure GPIO pins
    gpio_pin_configure_dt(&ain_1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&ain_2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&bin_1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&bin_2, GPIO_OUTPUT_INACTIVE);

    // Initialize PWM with zero duty cycle
    pwm_set_dt(&pwm_a, PWM_PERIOD, 0);
    pwm_set_dt(&pwm_b, PWM_PERIOD, 0);

    LOG_INF("Motor control initialized");
    return 0;
}

void motor_set_speed(float speed) {
    // Clamp speed to the range -1000 to +1000 
    // Increase range to have finer control, decrease range to have coarser control
    if (speed > 1000.0f) speed = 1000.0f;
    if (speed < -1000.0f) speed = -1000.0f;
    
    // Calculate PWM pulse based on speed percentage
    uint32_t pwm_pulse = (uint32_t)(fabsf(speed) * MAX_PULSE / 1000.0f);

    // Uncomment if motors seem to have a dead zone
    /*
    if (pwm_pulse > 0 && pwm_pulse < MIN_PULSE) {
        pwm_pulse = MIN_PULSE;  // Set a minimum effective threshold
    }
    */

    if (speed > 0) {
        // Forward direction
        gpio_pin_set_dt(&ain_1, 1);
        gpio_pin_set_dt(&ain_2, 0);
        gpio_pin_set_dt(&bin_1, 1);
        gpio_pin_set_dt(&bin_2, 0);
    } else if (speed < 0) {
        // Backward direction
        gpio_pin_set_dt(&ain_1, 0);
        gpio_pin_set_dt(&ain_2, 1);
        gpio_pin_set_dt(&bin_1, 0);
        gpio_pin_set_dt(&bin_2, 1);
    } else {
        // Stop motors
        gpio_pin_set_dt(&ain_1, 0);
        gpio_pin_set_dt(&ain_2, 0);
        gpio_pin_set_dt(&bin_1, 0);
        gpio_pin_set_dt(&bin_2, 0);
        pwm_pulse = 0;
    }
    
    // Set PWM for both motors
    pwm_set_dt(&pwm_a, PWM_PERIOD, pwm_pulse);
    pwm_set_dt(&pwm_b, PWM_PERIOD, pwm_pulse);
}