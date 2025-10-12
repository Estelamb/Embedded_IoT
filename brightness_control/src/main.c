#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensors/rgb_led/rgb_led.h"
#include "sensors/adc/adc.h"
#include "sensors/user_button/user_button.h"

// Phototransistor
struct adc_config pt = {
        .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
        .channel_id = 0,
        .resolution = 12,
        .gain = ADC_GAIN_1,
        .ref = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .vref_mv = 3300,
    };

// RGB LED
struct bus_rgb_led rgb_led = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(red), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(green), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(blue), gpios)
    },
    .pin_count = BUS_SIZE,
};

// User button
static struct user_button button = {
    .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
};

static void button_pressed_isr(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct user_button *btn = CONTAINER_OF(cb, struct user_button, callback);
    btn->pressed = true;
}


// Timer to read phototransistor periodically
void ticker_handler(struct k_timer *timer_id) {
    printk("Ticker fired\n");
}

K_TIMER_DEFINE(my_ticker, ticker_handler, NULL);

int main(void)
{
    printk("Embedded Platforms and Communications for IoT\n");
    printk("ETSIST - UPM - MUIoT 2025-2026\n\n");

    // Initialize RGB LED
    if (rgb_led_init(&rgb_led) != 0) {
        printk("Error: Failed to initialize RGB LED\n");
        return 0;
    }
    rgb_led_off(&rgb_led);

    // Initialize phototransistor
    if (adc_init(&pt) != 0) {
        printk("Error: Failed to initialize phototransistor\n");
        return 0;
    }

    // Initialize user button and attach ISR
    if (button_init(&button) != 0) {
        printk("Error: Failed to initialize button\n");
        return 0;
    }

    if (button_set_callback(&button, button_pressed_isr) != 0) {
        printk("Error: Failed to set button callback\n");
        return 0;
    }

    k_timer_start(&my_ticker, K_MSEC(2000), K_MSEC(2000));

    int count = 0;

    while (1) {
        if (button_was_pressed(&button)) {
            count++;
            rgb_led_write(&rgb_led, count);
            printk("Button pressed, RGB LED changed\n");
        }

        int32_t mv;
        adc_read_voltage(&mv);
        printk("Voltage: %d mV\n", mv);

        k_sleep(K_MSEC(1000));
    }
}
