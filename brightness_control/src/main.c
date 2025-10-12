#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensors/rgb_led/rgb_led.h"
#include "sensors/phototransistor/phototransistor.h"

#define INVALID_VOLTAGE -1
#define BUFFER_SIZE 1
#define RESOLUTION 12
#define CHANNEL_ID 5

struct bus_rgb_led rgb_led = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(red), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(green), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(blue), gpios)
    },
    .pin_count = BUS_SIZE,
};

static const struct device *pt = DEVICE_DT_GET(DT_NODELABEL(adc1));
static int16_t sample_buffer[BUFFER_SIZE];

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = CHANNEL_ID,
};

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

    // Initialize phototransistor (using external ADC config)
    if (phototransistor_init(pt, &channel_cfg) != 0) {
        printk("Error: Failed to initialize phototransistor\n");
        return 0;
    }

    int count = 0;

    while (1) {
        printk("Toggle LED + read phototransistor - Count: %d\n", count);

        rgb_led_write(&rgb_led, count);
        count++;

        float normalized = phototransistor_read_normalized(pt, RESOLUTION, CHANNEL_ID, sample_buffer, BUFFER_SIZE);
        int32_t voltage_mv = 0;
        phototransistor_read_voltage(pt, &channel_cfg, &voltage_mv, RESOLUTION, CHANNEL_ID, sample_buffer, BUFFER_SIZE);

        printk("Normalized value: %f, Voltage: %d mV\n", (double)normalized, voltage_mv);

        k_sleep(K_MSEC(1000));
    }
}
