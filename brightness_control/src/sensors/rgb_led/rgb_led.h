#ifndef RGB_LED_H
#define RGB_LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define BUS_SIZE 3

/**
 * @brief Structure representing an RGB LED connected via GPIO pins.
 */
struct bus_rgb_led {
    struct gpio_dt_spec pins[BUS_SIZE];  /**< GPIO pin specifications for R, G, B */
    size_t pin_count;                    /**< Number of pins in use (should be 3) */
};

/**
 * @brief Initialize all RGB LED pins as GPIO outputs.
 *
 * Each pin is configured as an output and set to inactive state (LED off).
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_init(struct bus_rgb_led *rgb_led);

/**
 * @brief Turn on all RGB LED colors (white light).
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_on(struct bus_rgb_led *rgb_led);

/**
 * @brief Turn off all RGB LED colors.
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_off(struct bus_rgb_led *rgb_led);

/**
 * @brief Write a value to the RGB LED pins.
 *
 * Each bit in `value` controls one color channel (bit 0 = red, bit 1 = green, bit 2 = blue).
 *
 * Example:
 * - 0x1 → Red
 * - 0x2 → Green
 * - 0x3 → Yellow (Red + Green)
 * - 0x7 → White (Red + Green + Blue)
 *
 * @param rgb_led Pointer to the RGB LED bus structure.
 * @param value Bitmask value to control the LED (0–7).
 * @return 0 on success, or a negative error code on failure.
 */
int rgb_led_write(struct bus_rgb_led *rgb_led, int value);

/**
 * @brief Set RGB LED color to red.
 */
int rgb_red(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to green.
 */
int rgb_green(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to blue.
 */
int rgb_blue(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to yellow (red + green).
 */
int rgb_yellow(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to cyan (green + blue).
 */
int rgb_cyan(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to magenta (red + blue).
 */
int rgb_magenta(struct bus_rgb_led *rgb_led);

/**
 * @brief Set RGB LED color to white (red + green + blue).
 */
int rgb_white(struct bus_rgb_led *rgb_led);

/**
 * @brief Turn off all colors (black/off).
 */
int rgb_black(struct bus_rgb_led *rgb_led);

#endif // RGB_LED_H
