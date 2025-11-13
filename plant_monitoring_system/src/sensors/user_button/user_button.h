/**
 * @file user_button.h
 * @brief GPIO-based user button handling with interrupt support.
 *
 * This driver configures a GPIO input pin for edge-triggered interrupts
 * (both rising and falling edges). The application must provide an ISR
 * callback to implement button press/release logic.
 */

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * @brief Structure representing a user button connected via GPIO.
 *
 * Contains the GPIO device specification and the callback structure
 * used by the Zephyr GPIO driver to handle interrupts.
 */
struct user_button {
    struct gpio_dt_spec spec;      /**< GPIO pin/device specification */
    struct gpio_callback callback; /**< GPIO callback descriptor */
};

/**
 * @brief Initialize a user button GPIO and configure edge interrupts.
 *
 * Configures the pin as input and enables interrupts on both rising
 * and falling edges to detect press and release transitions.
 *
 * @param button Pointer to a `user_button` structure.
 * @retval 0 If the initialization succeeded.
 * @retval Negative error code if GPIO configuration failed.
 */
int button_init(struct user_button *button);

/**
 * @brief Attach an ISR callback function to the button.
 *
 * The callback is executed in interrupt context whenever the configured
 * edge (rising/falling) is detected. The application should provide
 * a lightweight handler suitable for ISR execution.
 *
 * @param button Pointer to the `user_button` structure.
 * @param handler Function pointer to the GPIO ISR callback.
 * @retval 0 If the callback was successfully registered.
 * @retval Negative error code if registration failed.
 */
int button_set_callback(struct user_button *button, gpio_callback_handler_t handler);

#endif // USER_BUTTON_H
