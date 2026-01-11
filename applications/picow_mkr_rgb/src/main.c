#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>

#define NUM_LEDS 84

static struct led_rgb pixels[NUM_LEDS];

static const struct device *usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(mkr_rgb_board));

int main(void) {
    int ret;

    if (!device_is_ready(usb_acm)) {
        printk("CDC ACM device not ready\r\n");
        return 0;
    }

    // Enable USB device
    ret = usb_enable(NULL);
    if (ret != 0) {
        printk("Failed to enable USB\r\n");
        return 0;
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        pixels[i].r = 0x10;
        pixels[i].g = 0x00;
        pixels[i].b = 0x20;
    }

    led_strip_update_rgb(strip, pixels, NUM_LEDS);

    return 0;
}
