/**
 * @file rgb_led.c
 * @brief Implementation of RGB LED control using GPIO pins.
 *
 * This module provides initialization and color-setting functions for
 * a 3-channel RGB LED connected to GPIO pins.
 */

#include "rgb_led.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize all RGB LED GPIO pins.
 *
 * Checks if each GPIO port is ready, and configures all pins as outputs
 * with initial inactive (off) state.
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_init(struct bus_rgb_led *rgb_led) {
    for (size_t i = 0; i < rgb_led->pin_count; i++) {
        if (!device_is_ready(rgb_led->pins[i].port)) {
            printk("Error: GPIO device not ready for pin %d\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&rgb_led->pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("Error: Failed to configure output pin %d (code %d)\n", i, ret);
            return ret;
        }
    }

    printk("RGB LED initialized successfully\n");
    return 0;
}

/**
 * @brief Write a bitmask value to the RGB LED pins.
 *
 * Bit mapping:
 * - Bit 0 → Red
 * - Bit 1 → Green
 * - Bit 2 → Blue
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @param value Bitmask (0–7) controlling the color combination.
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_write(struct bus_rgb_led *rgb_led, int value) {
    for (size_t i = 0; i < rgb_led->pin_count; i++) {
        int pin_value = (value >> i) & 0x1;
        int ret = gpio_pin_set_dt(&rgb_led->pins[i], pin_value);
        if (ret != 0) {
            printk("Error: Failed to set pin %d (code %d)\n", i, ret);
            return ret;
        }
    }
    return 0;
}

/** @brief Turn on all RGB LED colors (white light). */
int rgb_led_on(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x7); }

/** @brief Turn off all RGB LED colors. */
int rgb_led_off(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x0); }

/** @brief Set LED color to red only. */
int rgb_red(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x1); }

/** @brief Set LED color to green only. */
int rgb_green(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x2); }

/** @brief Set LED color to blue only. */
int rgb_blue(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x4); }

/** @brief Set LED color to yellow (red + green). */
int rgb_yellow(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x3); }

/** @brief Set LED color to cyan (green + blue). */
int rgb_cyan(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x6); }

/** @brief Set LED color to purple (red + blue). */
int rgb_purple(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x5); }

/** @brief Set LED color to white (red + green + blue). */
int rgb_white(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x7); }

/** @brief Turn off all LED colors (black/off). */
int rgb_black(struct bus_rgb_led *rgb_led) { return rgb_led_write(rgb_led, 0x0); }
