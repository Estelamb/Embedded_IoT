/**
 * @file adc.c
 * @brief Implementation of ADC initialization and reading functions.
 *
 * This module provides routines to configure and read data from an
 * analog-to-digital converter (ADC) device using Zephyr's ADC API.
 */

#include "adc.h"

static const struct adc_config *adc_cfg_ptr = NULL;
static int16_t sample_buffer[BUFFER_SIZE];
static struct adc_channel_cfg channel_cfg;

/**
 * @brief Initialize the ADC with the given configuration.
 *
 * @param cfg Pointer to an adc_config structure with ADC parameters.
 * @return 0 on success, or a negative error code on failure.
 */
int adc_init(const struct adc_config *cfg) {
    if (!device_is_ready(cfg->dev)) {
        printk("ADC device not ready\n");
        return -ENODEV;
    }

    adc_cfg_ptr = cfg;

    channel_cfg.gain             = cfg->gain;
    channel_cfg.reference        = cfg->ref;
    channel_cfg.acquisition_time = cfg->acquisition_time;
    channel_cfg.channel_id       = cfg->channel_id;

    int ret = adc_channel_setup(cfg->dev, &channel_cfg);
    if (ret < 0) {
        printk("ADC channel setup failed: %d\n", ret);
        return ret;
    }

    printk("ADC initialized (dev=%s, ch=%d, res=%d)\n",
           cfg->dev->name, cfg->channel_id, cfg->resolution);

    return 0;
}

/**
 * @brief Read a raw ADC value from the configured channel.
 *
 * @param raw_val Pointer where the raw ADC sample will be stored.
 * @return 0 on success, or a negative error code on failure.
 */
int adc_read_raw(int16_t *raw_val) {
    if (!adc_cfg_ptr) {
        printk("ADC not initialized\n");
        return -EFAULT;
    }

    const struct adc_sequence sequence = {
        .channels    = BIT(adc_cfg_ptr->channel_id),
        .buffer      = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = adc_cfg_ptr->resolution,
    };

    int ret = adc_read(adc_cfg_ptr->dev, &sequence);
    if (ret < 0) {
        printk("ADC read failed: %d\n", ret);
        return ret;
    }

    *raw_val = sample_buffer[0];
    return 0;
}

/**
 * @brief Read a normalized ADC value between 0.0 and 1.0.
 *
 * @return Normalized floating-point value, or -1.0 on error.
 */
float adc_read_normalized(void) {
    int16_t raw;
    if (adc_read_raw(&raw) != 0) {
        return -1.0f;
    }
    return (float)raw / (float)((1 << adc_cfg_ptr->resolution) - 1);
}

/**
 * @brief Read the ADC voltage in millivolts.
 *
 * Converts the raw ADC sample to a voltage using the configured reference.
 *
 * @param out_mv Pointer where the result voltage (mV) will be stored.
 * @return 0 on success, or a negative error code on failure.
 */
int adc_read_voltage(int32_t *out_mv) {
    int16_t raw;
    int ret = adc_read_raw(&raw);
    if (ret < 0) return ret;

    *out_mv = (raw * adc_cfg_ptr->vref_mv) / ((1 << adc_cfg_ptr->resolution) - 1);
    return 0;
}
