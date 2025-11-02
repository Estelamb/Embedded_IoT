/**
 * @file adc.c
 * @brief ADC driver implementation for multi-channel usage in Zephyr.
 *
 * This module provides functions to initialize and read from ADC devices
 * using dynamic channel configuration. It supports reading raw ADC values,
 * normalized values, and computed voltages in millivolts.
 */

#include "adc.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

static int16_t sample_buffer[BUFFER_SIZE];

/**
 * @brief Initialize the ADC device by verifying its readiness.
 *
 * This function only checks if the device specified in the configuration
 * is ready. ADC channels are configured dynamically before each read.
 *
 * @param cfg Pointer to an ADC configuration structure.
 * @return 0 on success, or a negative error code.
 */
int adc_init(const struct adc_config *cfg)
{
    if (!device_is_ready(cfg->dev)) {
        printk("ADC device %s is not ready\n", cfg->dev->name);
        return -ENODEV;
    }

    printk("ADC device %s initialized successfully\n", cfg->dev->name);
    return 0;
}

/**
 * @brief Read a raw ADC value from the specified channel.
 *
 * This function dynamically configures the ADC channel using the provided
 * configuration and reads one raw sample into the provided buffer.
 *
 * @param cfg Pointer to an ADC configuration structure.
 * @param raw_val Pointer to store the raw ADC value.
 * @return 0 on success, or a negative error code.
 */
int adc_read_raw(const struct adc_config *cfg, int16_t *raw_val)
{
    struct adc_channel_cfg channel_cfg = {
        .gain             = cfg->gain,
        .reference        = cfg->ref,
        .acquisition_time = cfg->acquisition_time,
        .channel_id       = cfg->channel_id,
    };

    int ret = adc_channel_setup(cfg->dev, &channel_cfg);
    if (ret < 0) {
        printk("ADC channel setup failed (%d)\n", ret);
        return ret;
    }

    struct adc_sequence sequence = {
        .channels    = BIT(cfg->channel_id),
        .buffer      = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = cfg->resolution,
    };

    ret = adc_read(cfg->dev, &sequence);
    if (ret < 0) {
        printk("ADC read failed (%d)\n", ret);
        return ret;
    }

    *raw_val = sample_buffer[0];
    return 0;
}

/**
 * @brief Read a normalized ADC value (0.0 - 1.0) from the specified channel.
 *
 * This function reads the raw ADC value and normalizes it to the range 0.0 to 1.0.
 *
 * @param cfg Pointer to an ADC configuration structure.
 * @return Normalized ADC value as a float, or negative error code on failure.
 */
float adc_read_normalized(const struct adc_config *cfg)
{
    int16_t raw_val = 0;
    int ret = adc_read_raw(cfg, &raw_val);
    if (ret < 0) {
        return ret;
    }

    return (float)raw_val / ((1 << cfg->resolution) - 1);
}

/**
 * @brief Read the ADC value in millivolts from the specified channel.
 *
 * This function reads the raw ADC value and converts it to millivolts
 * based on the provided reference voltage.
 *
 * @param cfg Pointer to an ADC configuration structure.
 * @param out_mv Pointer to store the ADC voltage in millivolts.
 * @return 0 on success, or a negative error code.
 */
int adc_read_voltage(const struct adc_config *cfg, int32_t *out_mv)
{
    int16_t raw_val = 0;
    int ret = adc_read_raw(cfg, &raw_val);
    if (ret < 0) {
        return ret;
    }

    *out_mv = ((int32_t)raw_val * cfg->vref_mv) / ((1 << cfg->resolution) - 1);
    return 0;
}
