/**
 * @file main.c
 * @brief Main application for the simple brightness control system.
 *
 * This application reads ambient light using a phototransistor connected
 * to an ADC and controls an RGB LED accordingly. A user button toggles
 * the operating mode (OFF, NORMAL, BLUE). The main loop requests
 * brightness measurements periodically while a separate brightness
 * thread performs the ADC sampling and updates the shared context.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "brightness_thread.h"
#include "sensors/rgb_led/rgb_led.h"
#include "sensors/adc/adc.h"
#include "sensors/user_button/user_button.h"

#define LONG_PRESS_MS 1000  /**< Long press duration to toggle ON/OFF. */
#define BRIGHTNESS_REQUEST_INTERVAL_MS 2000 /**< Interval between brightness requests. */

/**
 * @brief System operating modes.
 *
 * - OFF_MODE: Device is turned off and LEDs are off.
 * - NORMAL_MODE: Normal operation where ambient brightness is measured
 *   and the LED color indicates the brightness level.
 * - BLUE_MODE: Special mode where the blue LED is shown continuously.
 */
typedef enum {
    OFF_MODE = 0,
    NORMAL_MODE,
    BLUE_MODE
} system_mode_t;


/* --- Peripheral configuration --- */
/**
 * @brief Phototransistor ADC configuration.
 *
 * This static configuration describes the ADC device, channel, and
 * reference voltage used to read the phototransistor voltage.
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
 * The `system_context` holds references to the ADC configuration,
 * current brightness value, and synchronization primitives. The mutex
 * is initialized by `start_brightness_thread()` when the thread starts.
 */
static struct system_context ctx = {
    .adc = &pt,
    .brightness = 0.0f,
    /* lock and semaphore initialized in start_brightness_thread() */
};

/**
 * @brief Button ISR callback.
 *
 * This interrupt service routine is called when the user button is
 * pressed. It sets the `pressed` flag inside the `user_button` structure
 * so the main loop can react to the event without performing work in
 * interrupt context.
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

/**
 * @brief Main application entry point.
 *
 * Initializes peripherals (RGB LED, ADC, user button), starts the
 * brightness thread, and runs the main loop which handles button input
 * and updates the LED color according to the selected mode and measured
 * brightness.
 *
 * @return This function does not return under normal operation.
 */
int main(void)
{
    printk("==== Simple Brightness Control System ====\n");

    /* Initialize peripherals */
    rgb_led_init(&rgb_led);
    rgb_led_off(&rgb_led);
    adc_init(&pt);
    button_init(&button);
    button_set_callback(&button, button_pressed_isr);

    /* Start the brightness thread which will perform ADC sampling on demand */
    start_brightness_thread(&ctx);

    /* Initial mode is OFF */
    system_mode_t mode = OFF_MODE;
    bool button_held = false;
    int64_t press_start = 0;
    int64_t last_measure_time = 0;

    while (1) {
        /* --- Button handling --- */
        if (gpio_pin_get_dt(&button.spec)) {
            if (!button_held) {
                button_held = true;
                press_start = k_uptime_get();
            } else if (k_uptime_get() - press_start > LONG_PRESS_MS) {
                /* Long press → toggle ON/OFF */
                if (mode == OFF_MODE) {
                    mode = NORMAL_MODE;
                    printk("System ON (NORMAL MODE)\n");
                } else {
                    mode = OFF_MODE;
                    rgb_led_off(&rgb_led);
                    printk("System OFF\n");
                }
                button_held = false;
                k_sleep(K_MSEC(500)); /* debounce */
            }
        } else if (button_held) {
            button_held = false;
            if (k_uptime_get() - press_start < LONG_PRESS_MS) {
                /* Short press → toggle NORMAL/BLUE */
                if (mode == NORMAL_MODE) {
                    mode = BLUE_MODE;
                    printk("Switched to BLUE MODE\n");
                } else if (mode == BLUE_MODE) {
                    mode = NORMAL_MODE;
                    printk("Switched to NORMAL MODE\n");
                }
            }
        }

        /* --- Request brightness measurement periodically --- */
        if (mode == NORMAL_MODE && (k_uptime_get() - last_measure_time) >= BRIGHTNESS_REQUEST_INTERVAL_MS) {
            request_brightness_measurement(&ctx);
            last_measure_time = k_uptime_get();
        }

        /* --- LED control according to mode and brightness --- */
        float brightness_copy = 0.0f;
        k_mutex_lock(&ctx.lock, K_FOREVER);
        brightness_copy = ctx.brightness;
        k_mutex_unlock(&ctx.lock);

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

        k_sleep(K_MSEC(100));
    }
}
