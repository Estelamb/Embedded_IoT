/**
 * @file board_led.c
 * @brief Implementation of LED control using GPIO pins.
 *
 * This module provides initialization and color-setting functions for
 * 3 board LEDs.
 */

#include "board_led.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize all LED GPIO pins.
 *
 * Checks if each GPIO port is ready, and configures all pins as outputs
 * with initial inactive (off) state.
 *
 * @param led Pointer to the LED bus structure.
 * @return 0 on success, or a negative error code on failure.
 */
int led_init(struct bus_led *led) {
    for (size_t i = 0; i < led->pin_count; i++) {
        if (!device_is_ready(led->pins[i].port)) {
            printk("Error: GPIO device not ready for pin %d\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&led->pins[i], GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            printk("Error: Failed to configure output pin %d (code %d)\n", i, ret);
            return ret;
        }
    }

    printk("LED initialized successfully\n");
    return 0;
}

/**
 * @brief Write a bitmask value to the LED pins.
 *
 * Bit mapping:
 * - Bit 0 → Red
 * - Bit 1 → Green
 * - Bit 2 → Blue
 *
 * @param led Pointer to the LED bus structure.
 * @param value Bitmask (0–7) controlling the color combination.
 * @return 0 on success, or a negative error code on failure.
 */
int led_write(struct bus_led *led, int value) {
    for (size_t i = 0; i < led->pin_count; i++) {
        int pin_value = (value >> i) & 0x1;
        int ret = gpio_pin_set_dt(&led->pins[i], pin_value);
        if (ret != 0) {
            printk("Error: Failed to set pin %d (code %d)\n", i, ret);
            return ret;
        }
    }
    return 0;
}

/** @brief Turn on all LED colors. */
int led_on(struct bus_led *led) { return led_write(led, 0x7); }

/** @brief Turn off all LED colors. */
int led_off(struct bus_led *led) { return led_write(led, 0x0); }

/** @brief Set LED color to red only. */
int red(struct bus_led *led) { return led_write(led, 0x1); }

/** @brief Set LED color to green only. */
int green(struct bus_led *led) { return led_write(led, 0x2); }

/** @brief Set LED color to blue only. */
int blue(struct bus_led *led) { return led_write(led, 0x4); }

/** @brief Set LED color to red and green only. */
int red_green(struct bus_led *led) { return led_write(led, 0x3); }

/** @brief Set LED color to green and blue only. */
int green_blue(struct bus_led *led) { return led_write(led, 0x6); }

/** @brief Set LED color to red and blue only. */
int red_blue(struct bus_led *led) { return led_write(led, 0x5); }