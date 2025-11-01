/**
 * @file main.c
 * @brief Main application for the interrupt-driven brightness control system.
 *
 * This application reads ambient light using a phototransistor connected
 * to an ADC and controls an RGB LED accordingly. A user button toggles
 * the operating mode (OFF, NORMAL, BLUE). Button presses are processed
 * via interrupts and deferred work. Press duration determines action.
 *
 * A brightness thread autonomously performs periodic brightness
 * measurements in NORMAL mode.
 *
 * Button behavior:
 * - Short press (< 1 s): Toggles between NORMAL and BLUE modes.
 * - Long press (≥ 1 s): Turns the system ON or OFF immediately.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "main.h"
#include "brightness_thread.h"
#include "sensors/rgb_led/rgb_led.h"
#include "sensors/adc/adc.h"
#include "sensors/user_button/user_button.h"

#define LONG_PRESS_MS 1000 /**< Long press duration threshold (in milliseconds). */
#define INITIAL_MODE NORMAL_MODE /**< Initial operating mode at startup. */

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

/* Semaphore to trigger brightness measurement in NORMAL mode */
static K_SEM_DEFINE(brightness_sem, 0, 1);

/**
 * @brief Shared context with the brightness thread.
 *
 * The @ref system_context structure holds references to the peripheral configuration.
 * Both the main thread and the brightness thread use this structure to
 * coordinate mode and brightness updates.
 */
static struct system_context ctx = {
    .phototransistor = &pt,
    .brightness = ATOMIC_INIT(0),
    .brightness_sem = &brightness_sem,
    .mode = ATOMIC_INIT(INITIAL_MODE),
};


/* --- Button timing & state -------------------------------------------------- */
static struct k_timer press_timer;
static struct k_work button_work;

static bool button_pressed = false;
static bool long_press_fired = false;
static bool ignore_release = false; 

/**
 * @brief Deferred work handler for processing button events.
 *
 * Determines if the event to process is a short or long press
 * based on the state of long_press_fired flag.
 */
static void button_work_handler(struct k_work *work)
{
    if (long_press_fired) {
        if (atomic_get(&ctx.mode) == NORMAL_MODE || atomic_get(&ctx.mode) == BLUE_MODE) {
            atomic_set(&ctx.mode, OFF_MODE);
            printk("System OFF\n");
        } else {
            atomic_set(&ctx.mode, NORMAL_MODE);
            k_sem_give(ctx.brightness_sem);
            printk("NORMAL MODE\n");
        }
    } else {
        if (atomic_get(&ctx.mode) == NORMAL_MODE) {
            atomic_set(&ctx.mode, BLUE_MODE);
            printk("BLUE MODE\n");
        } else if (atomic_get(&ctx.mode) == BLUE_MODE) {
            atomic_set(&ctx.mode, NORMAL_MODE);
            k_sem_give(ctx.brightness_sem);
            printk("NORMAL MODE\n");
        }
    }

    /* Reset flags after handling */
    long_press_fired = false;
}

/**
 * @brief Timer callback for detecting a long press.
 *
 * Invoked automatically after LONG_PRESS_MS milliseconds have passed
 * since the button was pressed.
 */
static void button_timer_handler(struct k_timer *timer)
{
    if (button_pressed && !long_press_fired) {
        long_press_fired = true;
        ignore_release = true;     // Ignore release after a long press
        k_work_submit(&button_work);
    }
}

/**
 * @brief ISR callback for button press/release events.
 *
 * Detects the press (active low) and release edges and manages
 * the press timer accordingly.
 */
static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get_dt(&button.spec)) {
        button_pressed = true;
        long_press_fired = false;
        ignore_release = false;
        k_timer_start(&press_timer, K_MSEC(LONG_PRESS_MS), K_NO_WAIT);
    } else {
        k_timer_stop(&press_timer);

        if (button_pressed && !long_press_fired && !ignore_release) {
            k_work_submit(&button_work);  // Short press
        }

        button_pressed = false;
        ignore_release = false;   // Reset after release
    }
}


/* --- Main Application -------------------------------------------------------- */

/**
 * @brief Main entry point for the brightness control system.
 *
 * Initializes peripherals (RGB LED, ADC, user button), starts the
 * brightness thread, and executes the LED update loop.
 *
 * Button input is interrupt-driven; all press logic is handled by ISR and workqueue.
 *
 * @return This function does not return under normal operation.
 */
int main(void)
{
    printk("==== Brightness Control System ====\n");
    int mode = INITIAL_MODE;
    int brightness = 0;

    /* Initialize peripherals */
    if (rgb_led_init(&rgb_led) || rgb_led_off(&rgb_led)) return -1;
    if (adc_init(&pt)) return -1;
    if (button_init(&button)) return -1;
    if (button_set_callback(&button, button_isr)) return -1;

    /* Button handling */
    k_timer_init(&press_timer, button_timer_handler, NULL);
    k_work_init(&button_work, button_work_handler);

    /* Start brightness measurement thread */
    start_brightness_thread(&ctx);

    printk("System ON (NORMAL MODE)\n");

    while (1) {
        mode = atomic_get(&ctx.mode);
        brightness = atomic_get(&ctx.brightness);

        switch (mode) {
            case OFF_MODE:
                rgb_led_off(&rgb_led);
                break;
            case BLUE_MODE:
                rgb_blue(&rgb_led);
                break;
            case NORMAL_MODE:
                if (brightness < 33) rgb_red(&rgb_led);
                else if (brightness < 66) rgb_yellow(&rgb_led);
                else rgb_green(&rgb_led);
                break;
        }

        k_sleep(K_MSEC(100));
    }
}
