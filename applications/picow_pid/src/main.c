/* main.c */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include "mpu6050.h"
#include "pid_controller.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define STACK_SIZE 1024

// Global device pointer
static const struct device *usb_acm;

/* Setter Getter Helpers */
#define UART_BUF_SIZE 32
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
            LOG_INF("PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", (double)kp, (double)ki, (double)kd);
            
            char response[64];
            snprintf(response, sizeof(response), 
                    "PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", 
                    (double)kp, (double)ki, (double)kd);
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
                (double)kp, (double)ki, (double)kd);
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
            //Echo the input back right away
            uart_fifo_fill(dev, &c, 1);
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
            else {
                // We’ve hit the buffer limit — take action!
                const char *warn_msg = "Command too long!. Ignoring input.\r\n";
                uart_fifo_fill(dev, (const uint8_t *)warn_msg, strlen(warn_msg));
                uart_rx_pos = 0; // Reset Buffer
            }
        }
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
    
    // Set UART callback for command processing
    uart_irq_callback_set(usb_acm, uart_cb);
    uart_irq_rx_enable(usb_acm);

    // Initialize balance controller 
    ret = balance_controller_init();
    if (ret != BALANCE_CONTROLLER_SUCCESS) {
        LOG_ERR("Balance controller initialization failed: %d\n", ret);
        return 0;
    }
    
    // Start balance controller thread
    balance_controller_start();

    // Send welcome message
    char welcome[] = "\r\n===== Self-Balancing Robot Control =====\r\n"
    "Type 'help' for available commands\r\n";
    uart_fifo_fill(usb_acm, (const uint8_t *)welcome, strlen(welcome));

    while (1) {
        k_sleep(K_FOREVER);
    }
    
    return 0;
}