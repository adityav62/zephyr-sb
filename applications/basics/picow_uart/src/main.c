#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>
//#include <zephyr/usb/usb_device.h> //no need for USB CDC since we are using UART pins
#include <zephyr/drivers/uart.h>

#define SLEEP_TIME_MS 1000


int main()
{
	//Initialize a UART device
	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if(!device_is_ready(uart_dev)){
		printk("UART device is not ready!!\n");
		return 0;
	}
	
	int i = 1;
	bool option = true;
	char buff[1024];
	buff[0] = '\0';

	while(option){
		printf("%d - Hello from the Zephyr console on RPi Pico W\n", (i % 10));
		k_sleep(K_MSEC(SLEEP_TIME_MS));
		printf("Good job on getting your first Zephyr Project working...\n\n\n");
		k_sleep(K_MSEC(2*SLEEP_TIME_MS));
		i++;
	}

	while(!option){
		printf("%d - Wow, you really went through all that setup just so I could say: ", i%10); 
		k_sleep(K_MSEC(SLEEP_TIME_MS));
		printf("'Hello, Human!'\n");
		k_sleep(K_MSEC(SLEEP_TIME_MS));
		printf("Now go touch some grass, tech wizard!\n\n\n");
		k_sleep(K_MSEC(2*SLEEP_TIME_MS));
		i++;
	}
	return 0;
}