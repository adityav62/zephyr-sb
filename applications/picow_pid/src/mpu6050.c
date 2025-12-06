/*mpu6050.c*/
#include "mpu6050.h"

LOG_MODULE_REGISTER(mpu6050, CONFIG_LOG_DEFAULT_LEVEL);

#define MPU6050_NODE DT_NODELABEL(mpu6050)
#define CALIBRATION_SAMPLES 500


static const struct device *mpu6050;

/* Variables to calculate bias of gyro and accelerometer */
static float gyro_bias_x = 0.0f;            // only pitch axis needed since our tilt happens about this axis
//static float gyro_bias_y = 0.0f;          // Uncomment if required for enhancing gyro calibration 
//static float gyro_bias_z = 0.0f;          // Uncomment if required for enhancing gyro calibration
//static float accel_bias_x = 0.0f;         // Uncomment if required for enhancing gyro calibration
static float accel_bias_y = 0.0f; 
static float accel_bias_z = 0.0f; 

/* Kalman filter variables */
static float Q_angle = 0.001f;              // Process noise variance for the angle
static float Q_gyro = 0.003f;               // Process noise variance for the gyro bias
static float R_measure = 0.03f;             // Measurement noise variance
static float angle = 0.0f;                  // Filtered angle
static float bias = 0.0f;                   // Gyro bias
static float P[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix

int mpu6050_begin(void){
    int ret;

    mpu6050 = DEVICE_DT_GET(MPU6050_NODE);

    // Check if device is ready
    if (!device_is_ready(mpu6050)) {
        //printk("MPU6050 device not found\n");
        return MPU6050_ERR_NOT_FOUND;
    }

    // Fetch data once to verify device is working
    ret = sensor_sample_fetch(mpu6050);
    if(ret <0){
        return MPU6050_ERR_INIT;
    }

    return MPU6050_SUCCESS;
}

int mpu6050_read(struct mpu6050_data *data){
    int ret;
    ret = sensor_sample_fetch(mpu6050);
    if(ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, data-> accel);
    if(ret < 0) {
        return ret;
    }

    ret = sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ, data-> gyro);
    if(ret < 0) {
        return ret;
    }

    return 0;
}

int calibrate_gyro(void) {
    struct mpu6050_data calib_data;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (mpu6050_read(&calib_data) < 0) {
            return -1;
        }

        sum_x += sensor_value_to_float(&calib_data.gyro[0]); // X-axis (pitch)
        sum_y += sensor_value_to_float(&calib_data.accel[1]); // Y-axis (Accel_Y)
        sum_z += sensor_value_to_float(&calib_data.accel[2]); // X-axis (Accel_Z)

        k_sleep(K_MSEC(2));  // 2 ms × 500 ≈ 1 second
    }

    gyro_bias_x = sum_x / CALIBRATION_SAMPLES;
    accel_bias_y = sum_y / CALIBRATION_SAMPLES;
    accel_bias_z = sum_z / CALIBRATION_SAMPLES;
    LOG_INF("Gyro X, Accel y, Accel Z bias calibrated: %.2f | %.2f | %.2f", (double)gyro_bias_x, (double)accel_bias_y, (double)accel_bias_z);
    return 0;
}

float calculate_angle(struct mpu6050_data *data, float dt) {
    float accel_y = sensor_value_to_float(&data->accel[1]);
    float accel_z = sensor_value_to_float(&data->accel[2]);
    float gyro_x = sensor_value_to_float(&data->gyro[0]) - gyro_bias_x;

    // Calculate angle in radians and convert to degrees
    double accel_angle = atan2f(accel_y, accel_z) * 57.2958f; // 180/PI = 57.2958

    return kalman_filter((float)accel_angle, gyro_x, dt);
}

float kalman_filter(float newAngle, float newRate, float dt) {
    // Prediction Step
    angle += (newRate - bias) * dt;
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;

    // Measurement Update Step
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle; // Angle difference
    angle += K[0] * y;
    bias += K[1] * y;

    float P00 = P[0][0];
    float P01 = P[0][1];

    P[0][0] = (1 - K[0]) * P00;
    P[0][1] = (1 - K[0]) * P01;
    P[1][0] = P[1][0] - K[1] * P00;
    P[1][1] = P[1][1] - K[1] * P01;

    return angle;
}