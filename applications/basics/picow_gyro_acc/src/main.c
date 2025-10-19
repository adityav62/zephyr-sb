#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#define MPU6050_NODE DT_NODELABEL(mpu6050)

int main(void) {
    const struct device *mpu6050 = DEVICE_DT_GET(MPU6050_NODE);
    const struct device *usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    
    int ret;

    if (!device_is_ready(usb_acm)) {
        printk("Failed to get USB UART device\n");
        return 0;
    }

    if (usb_enable(NULL)) {
        printk("Failed to enable USB\n");
        return 0;
    }

    if (!device_is_ready(mpu6050)) {
        printk("MPU6050 device not found\n");
        return 0;
    }

    while (1) {
        ret = sensor_sample_fetch(mpu6050);
        if (ret < 0) {
            printk("Failed to fetch sensor sample\n");
            k_sleep(K_SECONDS(1));
            continue;
        }

        ret = sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (ret < 0) {
            printk("Failed to get accelerometer data\n");
            k_sleep(K_SECONDS(1));
            continue;
        }

        ret = sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ, gyro);
        if (ret < 0) {
            printk("Failed to get gyroscope data\n");
            k_sleep(K_SECONDS(1));
            continue;
        }

        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer),
                           "\033[2J\033[H"
                           "Accel (m/s²)  |  Gyro (°/s)\r\n"
                           "--------------|-------------\r\n"
                           "X = %9.4f | X = %9.4f\r\n"
                           "Y = %9.4f | Y = %9.4f\r\n"
                           "Z = %9.4f | Z = %9.4f\r\n",
                           sensor_value_to_double(&accel[0]),
                           sensor_value_to_double(&gyro[0]),
                           sensor_value_to_double(&accel[1]),
                           sensor_value_to_double(&gyro[1]),
                           sensor_value_to_double(&accel[2]),
                           sensor_value_to_double(&gyro[2]));

        uart_fifo_fill(usb_acm, (const uint8_t *)buffer, len);

        k_sleep(K_MSEC(100));
    }
}
	