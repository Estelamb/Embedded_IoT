/**
 * @file board_led.h
 * @brief Interface for controlling an LED using GPIO pins.
 */

#ifndef BOARD_LED_H
#define BOARD_LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define BUS_SIZE 3  /**< Number of pins for the LED (R, G, B). */

/**
 * @brief Structure representing an LED connected via GPIO pins.
 */
struct bus_led {
    struct gpio_dt_spec pins[BUS_SIZE];  /**< GPIO pin specifications for R, G, B. */
    size_t pin_count;                    /**< Number of pins in use (should be 3). */
};

int led_init(struct bus_led *led);
int led_write(struct bus_led *led, int value);

int led_on(struct bus_led *led);
int led_off(struct bus_led *led);

int red(struct bus_led *led);
int green(struct bus_led *led);
int blue(struct bus_led *led);
int red_green(struct bus_led *led);
int green_blue(struct bus_led *led);
int red_blue(struct bus_led *led);

#endif // BOARD_LED_H
