#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <lvgl.h>
//#include <zephyr/drivers/display/display_st7735r.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
    const struct device *display_dev;
    lv_obj_t *label1;
    lv_obj_t *label2;
    lv_obj_t *label3;
    lv_obj_t *container;
    
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return;
    }
    
    LOG_INF("Display device initialized");
    
    // Set background color to white
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);
    
    // Create a container with oval shape
    container = lv_obj_create(lv_scr_act());
    // Make the oval wider but less tall to improve proportions
    lv_obj_set_size(container, 200, 100);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 0);
    
    // Style the container as a yellow oval - using direct RGB values
    lv_obj_set_style_radius(container, 50, LV_PART_MAIN); // Use specific radius instead of LV_RADIUS_CIRCLE
    // Try a different yellow definition - pure yellow in RGB
    lv_obj_set_style_bg_color(container, lv_color_make(255, 255, 0), LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN); // No border
    
    // Create first label with default font size
    label1 = lv_label_create(container);
    lv_label_set_text(label1, "Nett Hier!");
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -25); // Adjusted position
    
    // Set text color to black for better visibility on yellow background
    lv_obj_set_style_text_color(label1, lv_color_black(), LV_PART_MAIN);
    
    // Create second label with smaller font - first part
    label2 = lv_label_create(container);
    lv_label_set_text(label2, "Aber waren Sie schon mal");
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 5); // Adjusted position
    
    // Set text color to black and smaller font size
    lv_obj_set_style_text_color(label2, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label2, &lv_font_montserrat_8, LV_PART_MAIN); // Use smaller font
    
    // Create third label with smaller font - second part
    label3 = lv_label_create(container);
    lv_label_set_text(label3, "in Baden-Wuerttemberg?");
    lv_obj_align(label3, LV_ALIGN_CENTER, 0, 25); // Adjusted position
    
    // Set text color to black and smaller font size
    lv_obj_set_style_text_color(label3, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label3, &lv_font_montserrat_8, LV_PART_MAIN); // Use smaller font
    
    // Turn off display blanking to keep the display on
    display_blanking_off(display_dev);
    
    while (1) {
        // LVGL task handler needs to be called periodically
        lv_task_handler();
        k_sleep(K_MSEC(10));
    }
}