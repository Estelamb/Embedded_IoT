#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensors/rgb_led/rgb_led.h"
#include "sensors/adc/adc.h"
#include "sensors/button/button.h"

// Phototransistor
#define INVALID_VOLTAGE -1
#define BUFFER_SIZE 1
#define RESOLUTION 12
#define CHANNEL_ID 0

static const struct device *pt = DEVICE_DT_GET(DT_NODELABEL(adc1));
static int16_t sample_buffer[BUFFER_SIZE];

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = CHANNEL_ID,
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

/**
 * @brief ISR called when the button is pressed.
 */
static void button_pressed_isr(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct user_button *btn = CONTAINER_OF(cb, struct user_button, callback);
    btn->pressed = true;
}

Ticker tick;

void handler() {
    float normalized = adc_read_normalized(pt, RESOLUTION, CHANNEL_ID, sample_buffer, BUFFER_SIZE);
    int32_t voltage_mv = 0;
    adc_read_voltage(pt, &channel_cfg, &voltage_mv, RESOLUTION, CHANNEL_ID, sample_buffer, BUFFER_SIZE);
    
    printk("Normalized value: %f, Voltage: %d mV\n", (double)normalized, voltage_mv);
}


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
    if (adc_init(pt, &channel_cfg) != 0) {
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

    tick.attach(&handler, 2); // every 2 seconds

    int count = 0;

    while (1) {
        if (button_was_pressed(&button)) {
            rgb_led_write(&rgb_led, count);
            printk("Button pressed, RGB LED changed\n");
            count++;
        }

        k_sleep(K_MSEC(1000));
    }
}
