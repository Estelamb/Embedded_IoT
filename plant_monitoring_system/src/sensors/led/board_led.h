/**
 * @file board_led.h
 * @brief LED control interface using GPIO pins.
 *
 * This module provides a simple abstraction for controlling
 * single or multi-channel LEDs (e.g., RGB) via GPIO outputs.
 * It defines initialization and control functions for on/off
 * operations as well as combined color states.
 */

#ifndef BOARD_LED_H
#define BOARD_LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/** @brief Number of GPIO pins used to control an RGB LED (R, G, B). */
#define BUS_SIZE 3

/**
 * @brief LED descriptor structure for GPIO-controlled LEDs.
 *
 * Represents an LED or RGB LED using one or more GPIO pins.
 * Each element in @ref pins corresponds to a color channel.
 */
struct bus_led {
    struct gpio_dt_spec pins[BUS_SIZE];  /**< GPIO pin specifications for R, G, and B. */
    size_t pin_count;                    /**< Number of active pins (typically 3 for RGB). */
};

/**
 * @brief Initializes GPIO pins for LED control.
 *
 * Configures all pins defined in the LED structure for output mode
 * and ensures they are ready for use.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If initialization succeeded.
 * @retval -EINVAL If the LED configuration is invalid.
 * @retval -EIO If one or more GPIO devices could not be configured.
 */
int led_init(struct bus_led *led);

/**
 * @brief Writes a raw value to the LED GPIO pins.
 *
 * Each bit of the provided value corresponds to an LED channel.
 * Example: bit0=R, bit1=G, bit2=B.
 *
 * @param led Pointer to the LED descriptor structure.
 * @param value Bitmask specifying which LED channels are active.
 * @retval 0 If the write operation succeeded.
 */
int led_write(struct bus_led *led, int value);

/**
 * @brief Turns the LED on (activates all configured pins).
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int led_on(struct bus_led *led);

/**
 * @brief Turns the LED off (deactivates all configured pins).
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int led_off(struct bus_led *led);

/**
 * @brief Activates only the red LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red(struct bus_led *led);

/**
 * @brief Activates only the green LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int green(struct bus_led *led);

/**
 * @brief Activates only the blue LED channel.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int blue(struct bus_led *led);

/**
 * @brief Activates both red and green LED channels.
 *
 * Produces a yellow color on RGB LEDs.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red_green(struct bus_led *led);

/**
 * @brief Activates both green and blue LED channels.
 *
 * Produces a cyan color on RGB LEDs.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int green_blue(struct bus_led *led);

/**
 * @brief Activates both red and blue LED channels.
 *
 * Produces a magenta color on RGB LEDs.
 *
 * @param led Pointer to the LED descriptor structure.
 * @retval 0 If the operation succeeded.
 */
int red_blue(struct bus_led *led);

#endif // BOARD_LED_H
