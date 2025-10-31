/**
 * @file main.c
 * @brief Main application for the simple brightness control system.
 *
 * This application reads ambient light using a phototransistor connected
 * to an ADC and controls an RGB LED accordingly. A user button toggles
 * the operating mode (OFF, NORMAL, BLUE). The main loop updates the mode
 * and LED color, while the brightness thread autonomously performs
 * periodic brightness measurements when the system is in NORMAL mode.
 *
 * The button supports:
 * - Short press (< 1 s): Toggles between NORMAL and BLUE modes.
 * - Long press (≥ 1 s): Turns the system ON or OFF immediately,
 *   even if the button remains pressed.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "main.h"
#include "brightness_thread.h"
#include "sensors/rgb_led/rgb_led.h"
#include "sensors/adc/adc.h"
#include "sensors/user_button/user_button.h"

#define LONG_PRESS_MS 1000  /**< Long press duration threshold (in milliseconds). */

/* --- Peripheral configuration ------------------------------------------------ */

/**
 * @brief Phototransistor ADC configuration.
 */
static struct adc_config pt = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
    .channel_id = 0,
    .resolution = 12,
    .gain = ADC_GAIN_1,
    .ref = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .vref_mv = 3300,
};

/**
 * @brief RGB LED bus configuration.
 */
static struct bus_rgb_led rgb_led = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(red), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(green), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(blue), gpios)
    },
    .pin_count = BUS_SIZE,
};

/**
 * @brief User button configuration.
 */
static struct user_button button = {
    .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
};

/**
 * @brief Shared context with the brightness thread.
 *
 * The @ref system_context structure holds references to the ADC configuration,
 * current brightness value, and synchronization primitives.
 * Both the main thread and the brightness thread use this structure to
 * coordinate mode and brightness updates.
 */
static struct system_context ctx = {
    .adc = &pt,
    .brightness = 0.0f,
};

/* --- Button ISR -------------------------------------------------------------- */

/**
 * @brief Button interrupt service routine.
 *
 * Sets the @ref user_button.pressed flag so the main loop can process
 * the event outside of interrupt context.
 *
 * @param dev Pointer to the GPIO device (unused).
 * @param cb Pointer to the gpio_callback structure associated with the button.
 * @param pins Bitmask of the pins that triggered the interrupt (unused).
 */
static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct user_button *btn = CONTAINER_OF(cb, struct user_button, callback);
    btn->pressed = true;
}

/* --- Helper functions -------------------------------------------------------- */

/**
 * @brief Safely update the system mode with mutex protection.
 *
 * @param ctx Pointer to the shared system context.
 * @param new_mode New mode to set.
 */
static void set_system_mode(struct system_context *ctx, system_mode_t new_mode)
{
    k_mutex_lock(&ctx->lock, K_FOREVER);
    ctx->mode = new_mode;
    k_mutex_unlock(&ctx->lock);
}

/**
 * @brief Safely retrieve the current system mode with mutex protection.
 *
 * @param ctx Pointer to the shared system context.
 * @return The current system mode.
 */
static system_mode_t get_system_mode(struct system_context *ctx)
{
    system_mode_t mode_copy;
    k_mutex_lock(&ctx->lock, K_FOREVER);
    mode_copy = ctx->mode;
    k_mutex_unlock(&ctx->lock);
    return mode_copy;
}

/**
 * @brief Safely retrieve the current brightness value with mutex protection.
 *
 * @param ctx Pointer to the shared system context.
 * @return The last measured brightness percentage.
 */
static float get_brightness(struct system_context *ctx)
{
    float brightness_copy;
    k_mutex_lock(&ctx->lock, K_FOREVER);
    brightness_copy = ctx->brightness;
    k_mutex_unlock(&ctx->lock);
    return brightness_copy;
}

/* --- Main Application -------------------------------------------------------- */

/**
 * @brief Main entry point for the brightness control system.
 *
 * Initializes peripherals (RGB LED, ADC, user button), starts the
 * brightness thread, and executes the main control loop.
 *
 * The loop performs three main tasks:
 * 1. Detects short and long button presses.
 * 2. Updates the system mode accordingly.
 * 3. Controls the RGB LED color based on brightness and mode.
 *
 * @return This function does not return under normal operation.
 */
int main(void)
{
    printk("==== Simple Brightness Control System ====\n");

    /* --- Local state variables --- */
    system_mode_t mode = NORMAL_MODE;
    bool button_held = false;
    bool long_press_handled = false;
    int64_t press_start = 0;

    /* --- Peripheral initialization --- */
    rgb_led_init(&rgb_led);
    rgb_led_off(&rgb_led);
    adc_init(&pt);
    button_init(&button);
    button_set_callback(&button, button_pressed_isr);

    /* --- Initialize shared context BEFORE starting brightness thread --- */
    k_mutex_init(&ctx.lock);
    ctx.mode = mode;
    ctx.brightness = 0.0f;

    /* Start brightness measurement thread */
    start_brightness_thread(&ctx);

    printk("System ON (NORMAL MODE)\n");

    /* --- Main control loop --- */
    while (1) {
        bool pressed = gpio_pin_get_dt(&button.spec);

        /* --- Button handling --- */
        if (pressed) {
            if (!button_held) {
                /* Button was just pressed */
                button_held = true;
                long_press_handled = false;
                press_start = k_uptime_get();
            } else if (!long_press_handled) {
                /* Button is being held → check duration */
                int64_t press_duration = k_uptime_get() - press_start;

                if (press_duration >= LONG_PRESS_MS) {
                    /* Long press detected (trigger immediately) */
                    long_press_handled = true;

                    if (mode == OFF_MODE) {
                        mode = NORMAL_MODE;
                        printk("System ON (NORMAL MODE)\n");
                    } else {
                        mode = OFF_MODE;
                        rgb_led_off(&rgb_led);
                        printk("System OFF\n");
                    }

                    set_system_mode(&ctx, mode);
                }
            }
        } else if (button_held) {
            /* Button has just been released */
            button_held = false;

            if (!long_press_handled) {
                /* Short press detected */
                int64_t press_duration = k_uptime_get() - press_start;

                if (press_duration < LONG_PRESS_MS) {
                    if (mode == NORMAL_MODE) {
                        mode = BLUE_MODE;
                        printk("Switched to BLUE MODE\n");
                    } else if (mode == BLUE_MODE) {
                        mode = NORMAL_MODE;
                        printk("Switched to NORMAL MODE\n");
                    }

                    set_system_mode(&ctx, mode);
                }
            }
        }

        /* --- LED control according to mode and brightness --- */
        mode = get_system_mode(&ctx);
        float brightness_copy = get_brightness(&ctx);

        switch (mode) {
            case OFF_MODE:
                rgb_led_off(&rgb_led);
                break;

            case BLUE_MODE:
                rgb_blue(&rgb_led);
                break;

            case NORMAL_MODE:
                if (brightness_copy < 33.0f)
                    rgb_red(&rgb_led);
                else if (brightness_copy < 66.0f)
                    rgb_yellow(&rgb_led);
                else
                    rgb_green(&rgb_led);
                break;
        }

        /* Small delay to reduce CPU usage */
        k_sleep(K_MSEC(100));
    }
}
