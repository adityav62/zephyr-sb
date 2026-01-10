#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/usb/usb_device.h>
#include <string.h>

#define NUM_LEDS 84

static struct led_rgb pixels[NUM_LEDS];

static const struct device *strip =
    DEVICE_DT_GET(DT_NODELABEL(mkr_rgb_board));

int main(void)
{
    bool on = false;

    if (!device_is_ready(strip)) {
        printk("LED strip not ready\n");
        return 0;
    }

    /* Start from a known OFF state */
    memset(pixels, 0, sizeof(pixels));
    led_strip_update_rgb(strip, pixels, NUM_LEDS);

    while (1) {
        on = !on;

        if (on) {
            /* ALL ON (white, dim) */
            for (int i = 0; i < NUM_LEDS; i++) {
                pixels[i].r = 0x20;
                pixels[i].g = 0x20;
                pixels[i].b = 0x20;
            }
            printk("LEDs ON\r\n");
        } else {
            /* ALL OFF */
            memset(pixels, 0, sizeof(pixels));
            printk("LEDs OFF\r\n");
        }

        led_strip_update_rgb(strip, pixels, NUM_LEDS);

        k_sleep(K_SECONDS(5));
    }
}
