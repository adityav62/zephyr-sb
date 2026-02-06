/* pid_controller.h */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include "mpu6050.h"

// Error codes
#define BALANCE_CONTROLLER_SUCCESS 0
#define BALANCE_CONTROLLER_ERROR -1

typedef struct {
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
    float setpoint;    // Desired value
    float integral;    // Accumulated error
    float prev_error;  // Previous error for derivative calculation
    float output_min;  // Minimum output value
    float output_max;  // Maximum output value
} pid_controller_t;

// Initialize PID controller with gains and limits
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, 
              float output_min, float output_max);

// Update PID controller with new measurement and get output
float pid_update(pid_controller_t *pid, float measurement, float dt);

// Set new target value
void pid_set_setpoint(pid_controller_t *pid, float setpoint);

// Reset the integral term
void pid_reset(pid_controller_t *pid);

// Initialize the balance controller
int balance_controller_init(void);

// Start the balance controller thread
int balance_controller_start(void);

// Get the semaphore for synchronizing with new sensor data
struct k_sem *balance_controller_get_data_sem(void);

// Get the latest MPU6050 data
int balance_controller_get_mpu6050_data(struct mpu6050_data *data);

/*-----------------------Setter and Getter Functions-------------------------*/

// Get current PID parameters
void pid_get_parameters(pid_controller_t *pid, float *kp, float *ki, float *kd);

// Set new PID parameters
void pid_set_parameters(pid_controller_t *pid, float kp, float ki, float kd);

// Get access to the angle PID controller
pid_controller_t *balance_controller_get_angle_pid(void);

/*----------------------------------------------------------------------------*/

#endif /* PID_CONTROLLER_H */