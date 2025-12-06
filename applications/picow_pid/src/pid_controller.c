/* pid_controller.c */
#include "pid_controller.h"
#include "mpu6050.h"
#include "motor_control.h"

LOG_MODULE_REGISTER(pid_controller, CONFIG_LOG_DEFAULT_LEVEL);


#define STACK_SIZE 2048
#define CONTROLLER_PRIORITY 5

// Mutex to ensure thread safety
K_MUTEX_DEFINE(pid_mutex);

// Thread stack and data
K_THREAD_STACK_DEFINE(controller_stack, STACK_SIZE);
static struct k_thread controller_thread_data;

// Semaphore for data synchronization
K_SEM_DEFINE(data_sem, 0, 1);

// Sensor data storage
static struct mpu6050_data mpu6050_latest_data;

// PID controllers for balance
static pid_controller_t angle_pid;

// Controller thread function
static void controller_thread(void *p1, void *p2, void *p3)
{
    int ret;
    float angle, motor_output;
    uint32_t last_time = k_uptime_get();
    float dt;

    //For debugging time
    uint32_t loop_start, loop_end, elapsed;

    while (1) {
        loop_start = k_uptime_get();

        // Read MPU6050 data
        ret = mpu6050_read(&mpu6050_latest_data);
        uint32_t now = k_uptime_get(); //time immediately after sensor read

        if (ret < 0) {
            LOG_ERR("Failed to read MPU6050: %d", ret);
        } else {
            k_sem_give(&data_sem);

            //dt calculation
            dt = (now - last_time) / 1000.0f; // Convert ms to seconds
            last_time = now;

            // Calculate tilt angle
            angle = calculate_angle(&mpu6050_latest_data, dt);

            // PID control
            k_mutex_lock(&pid_mutex, K_FOREVER);
            motor_output = pid_update(&angle_pid, angle, dt);
            k_mutex_unlock(&pid_mutex);
            
            // Safety Threshold
            if(angle < 50.0f && angle > -50.0f){
                motor_set_speed(motor_output);
            }
            else{
                motor_set_speed(0.0f);
                pid_reset(&angle_pid); //prevents integral windup apparently
            }   
        }

        // ========TIMING DEBUG BLOCK START========== 
        //Comment when not needed: loop time debug
        
        /*
        loop_end = k_uptime_get();
        elapsed = loop_end - now;
        LOG_INF("Loop Start: %lu ms | Loop End: %lu ms", loop_start, loop_end);
        //LOG_INF("Loop time: %lu ms | dt: %.3f s | angle(rad): %.3f | output: %.2f",
                 elapsed, dt, angle, motor_output);
        

        // === Sleep to maintain periodic rate ===
        uint32_t control_elapsed = k_uptime_get() - loop_start; //total time elapsed from sensor acquisition to motor control
        LOG_INF("Elapsed vs Control Elapsed: %lu ms vs %lu ms", elapsed, control_elapsed);
        uint32_t sleep_time = 10 - control_elapsed;
        LOG_INF("Control Time: %lu ms| Sleep Time: %lu ms", control_elapsed, sleep_time);
        if (sleep_time > 0) {
            k_sleep(K_MSEC(sleep_time));
        }
        //k_sleep(K_MSEC(10));
        */
        // ========TIMING DEBUG BLOCK END==========
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
    pid_init(&angle_pid, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f);
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

    // Deadband to prevent overcorrection for small tilt -- may not be necessary when using Kalman filter 
    /*
    const float deadband_threshold = 0.25f;  // in degrees
    if (fabsf(error) < deadband_threshold) {
        return 0.0f;  // No motor output within deadband
    }
    */
    
    // Proportional term
    float p_term = pid->kp * error;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;    
    
    // Integral term without anti-windup
    //pid->integral += error * dt;
    //float i_term = pid->ki * pid->integral;
    
    //Integral term with antiwindup
    float integral_temp = pid->integral + error * dt;
    float i_term = pid->ki * integral_temp;

    // Calculate total output
    float output = p_term + i_term + d_term;

    //Anti-Windup Logic
    if(!((output > pid->output_max && error > 0) || (output < pid->output_min && error < 0))){
        pid->integral = integral_temp;
    }

    // Clamp Output
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
    k_mutex_lock(&pid_mutex, K_FOREVER);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    // Reset integral and error terms to avoid using stale values
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    k_mutex_unlock(&pid_mutex);
}

pid_controller_t *balance_controller_get_angle_pid(void) {
    return &angle_pid;
}