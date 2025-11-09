/**
 * @file rgb_led.h
 * @brief Interface for controlling an RGB LED using GPIO pins.
 */

#ifndef RGB_LED_H
#define RGB_LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define BUS_SIZE 3  /**< Number of pins for the RGB LED (R, G, B). */

/**
 * @brief Structure representing an RGB LED connected via GPIO pins.
 */
struct bus_rgb_led {
    struct gpio_dt_spec pins[BUS_SIZE];  /**< GPIO pin specifications for R, G, B. */
    size_t pin_count;                    /**< Number of pins in use (should be 3). */
};

int rgb_led_init(struct bus_rgb_led *rgb_led);
int rgb_led_write(struct bus_rgb_led *rgb_led, int value);

int rgb_led_on(struct bus_rgb_led *rgb_led);
int rgb_led_off(struct bus_rgb_led *rgb_led);

int rgb_red(struct bus_rgb_led *rgb_led);
int rgb_green(struct bus_rgb_led *rgb_led);
int rgb_blue(struct bus_rgb_led *rgb_led);
int rgb_yellow(struct bus_rgb_led *rgb_led);
int rgb_cyan(struct bus_rgb_led *rgb_led);
int rgb_purple(struct bus_rgb_led *rgb_led);
int rgb_white(struct bus_rgb_led *rgb_led);
int rgb_black(struct bus_rgb_led *rgb_led);

#endif // RGB_LED_H
