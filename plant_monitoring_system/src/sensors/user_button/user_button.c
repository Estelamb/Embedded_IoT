/**
 * @file user_button.c
 * @brief GPIO-based user button initialization with interrupt support.
 *
 * This module provides initialization and interrupt callback registration
 * for a user button input. It configures the GPIO pin as input with pull-up
 * and enables interrupts on both rising and falling edges. Actual button press
 * logic must be implemented by the application via the callback.
 */

#include "user_button.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize a user button GPIO with edge interrupts.
 *
 * Configures the pin as input with pull-up and enables interrupts
 * on both rising and falling edges.
 *
 * @param button Pointer to a `user_button` structure.
 * @return 0 on success, or a negative error code.
 */
int button_init(struct user_button *button)
{
    if (!button || !button->spec.port) {
        printk("Error: Invalid button configuration\n");
        return -EINVAL;
    }

    if (!device_is_ready(button->spec.port)) {
        printk("Error: Button device %s not ready\n", button->spec.port->name);
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&button->spec, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0) {
        printk("Error: Failed to configure button pin (%d)\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button->spec, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error: Failed to configure button interrupt (%d)\n", ret);
        return ret;
    }

    printk("User button initialized (edge-interrupt mode)\n");
    return 0;
}

/**
 * @brief Register a callback function for button interrupts.
 *
 * The handler will be executed in interrupt context when the configured
 * edge is detected (press or release).
 *
 * @param button Pointer to a `user_button` structure.
 * @param handler ISR callback function.
 * @return 0 on success, or a negative error code.
 */
int button_set_callback(struct user_button *button, gpio_callback_handler_t handler)
{
    if (!button || !handler) {
        return -EINVAL;
    }

    gpio_init_callback(&button->callback, handler, BIT(button->spec.pin));

    int ret = gpio_add_callback(button->spec.port, &button->callback);
    if (ret != 0) {
        printk("Error: Failed to add button callback (%d)\n", ret);
        return ret;
    }

    return 0;
}
