/* main.c */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include "mpu6050.h"
#include "pid_controller.h"
#include <zephyr/logging/log.h>
//----------Added for Setter and Getter-----
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//------------------------------------------
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define STACK_SIZE 1024
#define DISPLAY_PRIORITY 6

// Thread stack and data for display
K_THREAD_STACK_DEFINE(display_stack, STACK_SIZE);
static struct k_thread display_thread_data;

// Global device pointer
static const struct device *usb_acm;

/*----------Setter Getter Helpers---------*/
#define UART_BUF_SIZE 128
static uint8_t uart_rx_buf[UART_BUF_SIZE];
static int uart_rx_pos = 0;

static void process_command(const char *cmd) {
    LOG_INF("Processing command: %s", cmd);
    
    // Get the angle PID controller
    pid_controller_t *pid = balance_controller_get_angle_pid();
    
    // Command format: "pid kp ki kd"
    if (strncmp(cmd, "pid ", 4) == 0) {
        float kp, ki, kd;
        int items = sscanf(cmd + 4, "%f %f %f", &kp, &ki, &kd);
        
        if (items == 3) {
            pid_set_parameters(pid, kp, ki, kd);
            LOG_INF("PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
            
            char response[64];
            snprintf(response, sizeof(response), 
                    "PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", 
                    kp, ki, kd);
            uart_fifo_fill(usb_acm, (const uint8_t *)response, strlen(response));
        } else {
            char response[] = "Invalid format. Use: pid kp ki kd\r\n";
            uart_fifo_fill(usb_acm, (const uint8_t *)response, strlen(response));
        }
    }
    // Command: "getpid" - get current PID values
    else if (strcmp(cmd, "getpid") == 0) {
        float kp, ki, kd;
        pid_get_parameters(pid, &kp, &ki, &kd);
        
        char response[64];
        snprintf(response, sizeof(response), 
                "Current PID parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", 
                kp, ki, kd);
        uart_fifo_fill(usb_acm, (const uint8_t *)response, strlen(response));
    }
    // Command: "help" - show available commands
    else if (strcmp(cmd, "help") == 0) {
        char response[] = "Available commands:\r\n"
                         "  pid kp ki kd - Set PID parameters\r\n"
                         "  getpid       - Get current PID parameters\r\n"
                         "  help         - Show this help\r\n";
        uart_fifo_fill(usb_acm, (const uint8_t *)response, strlen(response));
    }
    else {
        char response[] = "Unknown command. Type 'help' for available commands.\r\n";
        uart_fifo_fill(usb_acm, (const uint8_t *)response, strlen(response));
    }
}

static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    
    if (!uart_irq_update(dev)) {
        return;
    }
    
    if (uart_irq_rx_ready(dev)) {
        // Read all available data
        while (uart_fifo_read(dev, &c, 1) == 1) {
            // Process the character
            if (c == '\r' || c == '\n') {
                // End of command, process it
                if (uart_rx_pos > 0) {
                    uart_rx_buf[uart_rx_pos] = '\0';  // Null-terminate
                    process_command((const char *)uart_rx_buf);
                    uart_rx_pos = 0;  // Reset buffer
                }
            } else if (uart_rx_pos < UART_BUF_SIZE - 1) {
                // Add to buffer
                uart_rx_buf[uart_rx_pos++] = c;
            }
        }
    }
}
/*-----------------------------------------------------------------*/

// Display thread function
static void display_thread(void *p1, void *p2, void *p3)
{
    char buffer[256];
    int len;
    struct mpu6050_data mpu6050_data;
    struct k_sem *data_sem = balance_controller_get_data_sem();

    while (1) {
        // Wait for new sensor data
        k_sem_take(data_sem, K_FOREVER);
        
        // Get the latest MPU6050 data
        balance_controller_get_mpu6050_data(&mpu6050_data);
        
        // Format and display the data
        len = snprintf(buffer, sizeof(buffer),
                      "\033[2J\033[H"
                      "Accel (m/s²)  |  Gyro (°/s)\r\n"
                      "--------------|-------------\r\n"
                      "X = %9.4f | X = %9.4f\r\n"
                      "Y = %9.4f | Y = %9.4f\r\n"
                      "Z = %9.4f | Z = %9.4f\r\n",
                      sensor_value_to_double(&mpu6050_data.accel[0]),
                      sensor_value_to_double(&mpu6050_data.gyro[0]),
                      sensor_value_to_double(&mpu6050_data.accel[1]),
                      sensor_value_to_double(&mpu6050_data.gyro[1]),
                      sensor_value_to_double(&mpu6050_data.accel[2]),
                      sensor_value_to_double(&mpu6050_data.gyro[2]));
        
        uart_fifo_fill(usb_acm, (const uint8_t *)buffer, len);
        
        // Update display at a lower rate
        k_sleep(K_MSEC(100));
    }
}

int main(void)
{
    int ret;
    
    // Get USB ACM device
    usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(usb_acm)) {
        LOG_ERR("Failed to get USB UART device\n");
        return 0;
    }
    
    // Initialize USB
    if (usb_enable(NULL)) {
        LOG_ERR("Failed to enable USB\n");
        return 0;
    }
    
    // Wait for DTR signal
    uint32_t dtr = 0;
    while (!dtr) {
        uart_line_ctrl_get(usb_acm, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
    
    /*--------------Setter Getter additions----------*/
    // Set UART callback for command processing
    uart_irq_callback_set(usb_acm, uart_cb);
    uart_irq_rx_enable(usb_acm);
    /*----------------------------------------------*/

    // Initialize balance controller (which initializes MPU6050)
    ret = balance_controller_init();
    if (ret != BALANCE_CONTROLLER_SUCCESS) {
        LOG_ERR("Balance controller initialization failed: %d\n", ret);
        return 0;
    }
    
    // Start balance controller thread
    balance_controller_start();
    
    // Create display thread
    /*
    k_thread_create(&display_thread_data, display_stack,
                    STACK_SIZE, display_thread,
                    NULL, NULL, NULL,
                    DISPLAY_PRIORITY, 0, K_NO_WAIT);
    */
    // Main thread can do other work or just sleep

    // Send welcome message
    char welcome[] = "\r\n===== Self-Balancing Robot Control =====\r\n"
    "Type 'help' for available commands\r\n";
    uart_fifo_fill(usb_acm, (const uint8_t *)welcome, strlen(welcome));

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    
    return 0;
}