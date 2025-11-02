/**
 * @file user_button.h
 * @brief Interface for GPIO-based user button handling with interrupt support.
 *
 * This driver configures a GPIO input pin for interrupt-triggered events
 * (both rising and falling edges). Button press logic must be implemented
 * by the application through a provided ISR callback.
 */

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * @brief Structure representing a user button connected via GPIO.
 *
 * Holds the GPIO device specification and ISR callback structure.
 */
struct user_button {
    struct gpio_dt_spec spec;      /**< GPIO device specification */
    struct gpio_callback callback; /**< GPIO callback structure */
};

/**
 * @brief Initialize the user button GPIO and configure edge interrupts.
 *
 * The pin is configured as input and interrupt triggers are enabled on
 * both rising and falling edges to detect press and release transitions.
 *
 * @param button Pointer to the user_button structure.
 * @return 0 on success, or a negative error code on failure.
 */
int button_init(struct user_button *button);

/**
 * @brief Attach an ISR callback function to the button interrupt.
 *
 * The handler function is invoked directly in interrupt context on
 * detected edge transitions.
 *
 * @param button Pointer to the user_button structure.
 * @param handler Function pointer to the interrupt handler.
 * @return 0 on success, or a negative error code on failure.
 */
int button_set_callback(struct user_button *button, gpio_callback_handler_t handler);

#endif // USER_BUTTON_H
