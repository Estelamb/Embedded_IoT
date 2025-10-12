#include "user_button.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Initialize the user button GPIO and interrupt.
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
 * @brief Attach ISR callback to button interrupt.
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
 */
bool button_was_pressed(struct user_button *button)
{
    if (button->pressed) {
        button->pressed = false;
        return true;
    }
    return false;
}
