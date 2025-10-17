/**
 * @file user_button.h
 * @brief Interface for handling a user button via GPIO with interrupt support.
 */

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * @brief Structure representing a user button connected via GPIO.
 *
 * This structure holds the device tree specification, callback configuration,
 * and a volatile flag indicating whether the button was pressed.
 */
struct user_button {
    struct gpio_dt_spec spec;        /**< GPIO device specification. */
    struct gpio_callback callback;   /**< GPIO callback structure. */
    volatile bool pressed;           /**< Flag indicating button press event. */
};

/**
 * @brief Initialize the user button GPIO and configure interrupt triggering.
 *
 * @param button Pointer to the user_button structure.
 * @return 0 on success, or a negative error code on failure.
 */
int button_init(struct user_button *button);

/**
 * @brief Attach an ISR callback function to the button interrupt.
 *
 * @param button Pointer to the user_button structure.
 * @param handler Function pointer to the interrupt handler.
 * @return 0 on success, or a negative error code on failure.
 */
int button_set_callback(struct user_button *button, gpio_callback_handler_t handler);

/**
 * @brief Check and clear the button pressed flag.
 *
 * @param button Pointer to the user_button structure.
 * @return true if the button was pressed, false otherwise.
 */
bool button_was_pressed(struct user_button *button);

#endif // USER_BUTTON_H
