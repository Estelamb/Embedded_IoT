/**
 * @file board_led.c
 * @brief Implementation of LED control using GPIO pins.
 *
 * This module provides initialization and color-setting functions
 * for LEDs connected via GPIO. It supports RGB combinations through
 * bitmask-based control.
 */

#include "board_led.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initializes all GPIO pins used by the LED.
 *
 * Each pin defined in the @ref bus_led structure is verified to ensure
 * the corresponding GPIO device is ready. The pins are then configured
 * as outputs and set to an inactive (off) state by default.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If initialization succeeded.
 * @retval -ENODEV If a GPIO device is not ready.
 * @retval Other Negative error code from @ref gpio_pin_configure_dt.
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
 * @brief Writes a bitmask value to the LED GPIO pins.
 *
 * Each bit in the input value corresponds to one LED channel:
 * - Bit 0 → Red
 * - Bit 1 → Green
 * - Bit 2 → Blue
 *
 * Example:
 * - `0x1` → Red ON
 * - `0x3` → Red + Green ON (Yellow)
 * - `0x7` → All ON (White)
 *
 * @param led Pointer to the LED descriptor structure.
 * @param value Bitmask (0–7) controlling the color combination.
 * @retval 0 If the operation succeeded.
 * @retval -EIO If a GPIO write failed.
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

/**
 * @brief Turns on all LED channels.
 *
 * Equivalent to setting the RGB bitmask to 0x7 (all colors active).
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int led_on(struct bus_led *led) { return led_write(led, 0x7); }

/**
 * @brief Turns off all LED channels.
 *
 * Equivalent to setting the RGB bitmask to 0x0 (all off).
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int led_off(struct bus_led *led) { return led_write(led, 0x0); }

/**
 * @brief Activates only the red LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red(struct bus_led *led) { return led_write(led, 0x1); }

/**
 * @brief Activates only the green LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int green(struct bus_led *led) { return led_write(led, 0x2); }

/**
 * @brief Activates only the blue LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int blue(struct bus_led *led) { return led_write(led, 0x4); }

/**
 * @brief Activates red and green LED channels.
 *
 * Produces yellow on an RGB LED.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red_green(struct bus_led *led) { return led_write(led, 0x3); }

/**
 * @brief Activates green and blue LED channels.
 *
 * Produces cyan on an RGB LED.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int green_blue(struct bus_led *led) { return led_write(led, 0x6); }

/**
 * @brief Activates red and blue LED channels.
 *
 * Produces magenta on an RGB LED.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red_blue(struct bus_led *led) { return led_write(led, 0x5); }
