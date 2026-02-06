/*mpu6050.h*/
#ifndef MPU6050_H
#define MPU6050_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/*Error Codes specific to MPU6050*/
#define MPU6050_SUCCESS 0
#define MPU6050_ERR_NOT_FOUND -ENODEV //Device not found
#define MPU6050_ERR_INIT -EINVAL //Initialization Error

struct mpu6050_data{
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
};

/*
 * @brief Initialize the MPU6050 sensor
 *
 * @return MPU6050_SUCCESS on success
 * @return MPU6050_ERR_NOT_FOUND if device is not found
 * @return MPU6050_ERR_INIT if initialization fails
 */

//Function prototypes for MPU6050 functionality
int mpu6050_begin(void);
int mpu6050_read(struct mpu6050_data *data);

#endif