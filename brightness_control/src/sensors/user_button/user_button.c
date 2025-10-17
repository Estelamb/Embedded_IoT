/**
 * @file user_button.c
 * @brief Implementation of GPIO-based user button handling.
 *
 * This module provides initialization, interrupt callback registration,
 * and press-detection utilities for a user button connected via GPIO.
 */

#include "user_button.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize the user button GPIO and configure interrupt triggering.
 *
 * Configures the GPIO pin as input and sets it to trigger interrupts
 * on the active edge. Also clears the internal `pressed` flag.
 *
 * @param button Pointer to the user_button structure.
 * @return 0 on success, or a negative error code on failure.
 */
int button_init(struct user_button *button)
{
    if (!device_is_ready(button->spec.port)) {
        printk("Error: Button device %s is not ready\n", button->spec.port->name);
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&button->spec, GPIO_INPUT);
    if (ret != 0) {
        printk("Error: Failed to configure button pin (%d)\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button->spec, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error: Failed to configure button interrupt (%d)\n", ret);
        return ret;
    }

    button->pressed = false;

    printk("User button initialized successfully\n");
    return 0;
}

/**
 * @brief Attach an ISR callback function to the button interrupt.
 *
 * The provided handler will be called whenever the button is pressed.
 *
 * @param button Pointer to the user_button structure.
 * @param handler Function pointer to the interrupt handler.
 * @return 0 on success, or a negative error code on failure.
 */
int button_set_callback(struct user_button *button, gpio_callback_handler_t handler)
{
    gpio_init_callback(&button->callback, handler, BIT(button->spec.pin));

    int ret = gpio_add_callback(button->spec.port, &button->callback);
    if (ret != 0) {
        printk("Error: Failed to add button callback (%d)\n", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Check and clear the button pressed flag.
 *
 * If the flag is set (indicating a press was detected), it is reset and `true` is returned.
 * Otherwise, `false` is returned without modifying the flag.
 *
 * @param button Pointer to the user_button structure.
 * @return `true` if the button was pressed, `false` otherwise.
 */
bool button_was_pressed(struct user_button *button)
{
    if (button->pressed) {
        button->pressed = false;
        return true;
    }
    return false;
}
