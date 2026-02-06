/* pid_controller.c */
#include "pid_controller.h"
#include "mpu6050.h"
#include "motor_control.h"


LOG_MODULE_REGISTER(pid_controller, CONFIG_LOG_DEFAULT_LEVEL);

#define STACK_SIZE 2048
#define CONTROLLER_PRIORITY 5

// Thread stack and data
K_THREAD_STACK_DEFINE(controller_stack, STACK_SIZE);
static struct k_thread controller_thread_data;

// Semaphore for data synchronization
K_SEM_DEFINE(data_sem, 0, 1);

// Sensor data storage
static struct mpu6050_data mpu6050_latest_data;

// PID controllers for balance
static pid_controller_t angle_pid;

// Variable to calculate bias of gyro
static float gyro_bias_x = 0.0f;  // only pitch axis needed

// === GYRO CALIBRATION ===
#define CALIBRATION_SAMPLES 500
static int calibrate_gyro(void) {
    struct mpu6050_data calib_data;
    float sum = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (mpu6050_read(&calib_data) < 0) {
            return -1;
        }

        sum += sensor_value_to_double(&calib_data.gyro[0]); // X-axis (pitch)

        k_sleep(K_MSEC(2));  // 2 ms × 500 ≈ 1 second
    }

    gyro_bias_x = sum / CALIBRATION_SAMPLES;
    LOG_INF("Gyro X bias calibrated: %.5f", gyro_bias_x);
    return 0;
}

// Function to calculate tilt angle from accelerometer and gyroscope data
static float calculate_angle(struct mpu6050_data *data, float dt) {
    // Simple angle calculation from accelerometer data
    // This is a basic implementation - you might want to use a complementary or Kalman filter
    //float accel_x = sensor_value_to_double(&data->accel[0]);
    float accel_y = sensor_value_to_float(&data->accel[1]);
    float accel_z = sensor_value_to_float(&data->accel[2]);
    //float gyro_x = sensor_value_to_double(&data->gyro[0]); //X axis is the pitch axis
    float gyro_x = sensor_value_to_float(&data->gyro[0]) - gyro_bias_x;

    static float filtered_angle = 0;
    
    // Calculate angle in radians and convert to degrees
    float accel_angle = atan2f(accel_y, accel_z) * 57.29578f; // 180/PI = 57.29578
    float gyro_angle = filtered_angle + (gyro_x * dt);
    filtered_angle = (0.98f * gyro_angle) + (0.02f * accel_angle);
    
    return filtered_angle;
}

// Controller thread function
static void controller_thread(void *p1, void *p2, void *p3)
{
    int ret;
    float angle, motor_output;
    int64_t last_time = k_uptime_get();

    while (1) {
        int64_t now = k_uptime_get();
        float dt = (now - last_time) / 1000.0f; // Convert ms to seconds
        last_time = now;

        // Read MPU6050 data
        ret = mpu6050_read(&mpu6050_latest_data);
        if (ret < 0) {
            LOG_ERR("Failed to read MPU6050: %d", ret);
        } else {
            k_sem_give(&data_sem);

            // Calculate tilt angle
            angle = calculate_angle(&mpu6050_latest_data, dt);

            // PID control
            motor_output = pid_update(&angle_pid, angle, dt);

            // Log values to check output
            LOG_INF("Angle: %.2f | Output: %.2f", angle, motor_output);

            // Apply to motors
            if(angle > 15.0f && angle < -15.0f){
                motor_set_speed(0.0f);
            }
            else{
                motor_set_speed(motor_output);
            }
        }

        k_sleep(K_MSEC(10));
    }
}


int balance_controller_init(void)
{
    int ret;
    
    // Initialize MPU6050
    ret = mpu6050_begin();
    if (ret != MPU6050_SUCCESS) {
        LOG_ERR("MPU6050 initialization failed: %d", ret);
        return BALANCE_CONTROLLER_ERROR;
    }
    
    // === GYRO CALIBRATION ===
    ret = calibrate_gyro();
    if (ret < 0) {
        LOG_ERR("Gyro calibration failed");
        return BALANCE_CONTROLLER_ERROR;
    }

    // Initialize motor control
    ret = motor_init();
    if (ret != 0) {
        LOG_ERR("Motor initialization failed: %d", ret);
        return BALANCE_CONTROLLER_ERROR;
    }
    
    // Initialize PID controller
    pid_init(&angle_pid, 15.0f, 0.5f, 0.2f, -100.0f, 100.0f);
    pid_set_setpoint(&angle_pid, 0.0f);  // Target is upright position
    
    return BALANCE_CONTROLLER_SUCCESS;
}

int balance_controller_start(void)
{
    // Create controller thread
    k_thread_create(&controller_thread_data, controller_stack,
                    STACK_SIZE, controller_thread,
                    NULL, NULL, NULL,
                    CONTROLLER_PRIORITY, 0, K_NO_WAIT);
    
    return BALANCE_CONTROLLER_SUCCESS;
}

struct k_sem *balance_controller_get_data_sem(void)
{
    return &data_sem;
}

int balance_controller_get_mpu6050_data(struct mpu6050_data *data)
{
    if (data == NULL) {
        return BALANCE_CONTROLLER_ERROR;
    }
    
    // Copy the latest data
    memcpy(data, &mpu6050_latest_data, sizeof(struct mpu6050_data));
    
    return BALANCE_CONTROLLER_SUCCESS;
}

// PID controller implementation
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, 
              float output_min, float output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

float pid_update(pid_controller_t *pid, float measurement, float dt) {
    // Calculate error
    float error = pid->setpoint - measurement;

    // === Deadband to prevent overcorrection for small tilt ===
    const float deadband_threshold = 0.0f;  // in degrees
    if (fabsf(error) < deadband_threshold) {
        return 0.0f;  // No motor output within deadband
    }
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    // Save error for next iteration
    pid->prev_error = error;
    
    return output;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint) {
    pid->setpoint = setpoint;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* Setter and Getter function implementations */

void pid_get_parameters(pid_controller_t *pid, float *kp, float *ki, float *kd) {
    if (kp) *kp = pid->kp;
    if (ki) *ki = pid->ki;
    if (kd) *kd = pid->kd;
}

void pid_set_parameters(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    // Reset integral and error terms to avoid using stale values
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

pid_controller_t *balance_controller_get_angle_pid(void) {
    return &angle_pid;
}