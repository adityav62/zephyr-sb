#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#define STRIP_NUM_PIXELS 84

#define DELAY_TIME K_MSEC(40)

#define SAMPLE_LED_BRIGHTNESS 16

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
    RGB(SAMPLE_LED_BRIGHTNESS, 0x00, 0x00), /* red */
    RGB(0x00, SAMPLE_LED_BRIGHTNESS, 0x00), /* green */
    RGB(0x00, 0x00, SAMPLE_LED_BRIGHTNESS), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(DT_NODELABEL(mkr_rgb_board));

int main(void)
{
    size_t color = 0;
    int rc;

    if (device_is_ready(strip)) {
        printk("Found LED strip device %s", strip->name);
    } else {
        printk("LED strip device %s is not ready", strip->name);
        return 0;
    }

    printk("Displaying pattern on strip");
    while (1) {
        for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
            memset(&pixels, 0x00, sizeof(pixels));
            memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));

            rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            if (rc) {
                printk("couldn't update strip: %d", rc);
            }

            k_sleep(DELAY_TIME);
        }

        color = (color + 1) % ARRAY_SIZE(colors);
    }

    return 0;
}