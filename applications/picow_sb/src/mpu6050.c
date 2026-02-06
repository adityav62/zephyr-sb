/*mpu6050.c*/
#include "mpu6050.h"

#define MPU6050_NODE DT_NODELABEL(mpu6050)

static const struct device *mpu6050;

int mpu6050_begin(void){
    int ret;

    mpu6050 = DEVICE_DT_GET(MPU6050_NODE);

    //check if device is ready
    if (!device_is_ready(mpu6050)) {
        //printk("MPU6050 device not found\n");
        return MPU6050_ERR_NOT_FOUND;
    }

    //fetch a single data to verify device working
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