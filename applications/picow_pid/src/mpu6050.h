/*mpu6050.h*/
#ifndef MPU6050_H
#define MPU6050_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

/*Error Codes specific to MPU6050*/
#define MPU6050_SUCCESS 0
#define MPU6050_ERR_NOT_FOUND -ENODEV //Device not found
#define MPU6050_ERR_INIT -EINVAL //Initialization Error

// Structure to hold raw accelerometer and gyroscope values from the MPU6050
struct mpu6050_data{
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
};

// Initialize the MPU6050
int mpu6050_begin(void);

// Read MPU6050 Sensor Data
int mpu6050_read(struct mpu6050_data *data);

// Calibrate the MPU6050's  Gyroscope and Accelerometer */
int calibrate_gyro(void);

// Calculate Angle using raw Gyroscope and Accelerometer Data
float calculate_angle(struct mpu6050_data *data, float dt);


// Improve angle estimate using Kalman Filter
float kalman_filter(float newAngle, float newRate, float dt);
#endif