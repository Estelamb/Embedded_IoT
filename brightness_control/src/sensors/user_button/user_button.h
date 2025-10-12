#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/**
 * @brief Structure representing a user button connected via GPIO.
 *
 * This structure holds the device tree specification of the button,
 * the callback object, and a flag to indicate button press events.
 */
struct user_button {
    struct gpio_dt_spec spec;        /**< GPIO device specification */
    struct gpio_callback callback;   /**< GPIO callback structure */
    volatile bool pressed;           /**< Flag indicating button press event */
};

/**
 * @brief Initialize the user button.
 *
 * Configures the button as an input with an interrupt on edge-to-active.
 * The ISR must be attached later by the user via `button_set_callback()`.
 *
 * @param button Pointer to the user_button structure.
 * @return 0 on success, or a negative error code on failure.
 */
int button_init(struct user_button *button);

/**
 * @brief Attach an ISR callback function for button interrupts.
 *
 * The provided ISR will be called when the button is pressed.
 *
 * @param button Pointer to the user_button structure.
 * @param handler ISR function pointer.
 * @return 0 on success, or a negative error code on failure.
 */
int button_set_callback(struct user_button *button,
                        gpio_callback_handler_t handler);

/**
 * @brief Check if the button was pressed and clear the flag.
 *
 * This function checks the `pressed` flag, and if true, resets it to false.
 *
 * @param button Pointer to the user_button structure.
 * @return true if the button was pressed, false otherwise.
 */
bool button_was_pressed(struct user_button *button);

#endif // USER_BUTTON_H
