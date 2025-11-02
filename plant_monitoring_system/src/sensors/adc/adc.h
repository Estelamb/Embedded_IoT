/**
 * @file adc.h
 * @brief Interface for ADC sampling using Zephyr drivers.
 *
 * This module defines structures and functions for initializing and reading from
 * ADC devices in Zephyr. It allows dynamic channel configuration to support
 * multiple sensors on the same ADC device.
 */

#ifndef ADC_H
#define ADC_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#define BUFFER_SIZE 1 /**< ADC sample buffer size (1 sample). */

/**
 * @brief ADC configuration structure.
 *
 * This structure contains the necessary information to configure and read from
 * an ADC channel. Each sensor using the ADC should have its own instance.
 */
struct adc_config {
    const struct device *dev;      /**< Pointer to the ADC device. */
    uint8_t channel_id;            /**< ADC channel number. */
    uint8_t resolution;            /**< ADC resolution in bits. */
    enum adc_gain gain;            /**< ADC gain setting. */
    enum adc_reference ref;        /**< ADC reference source. */
    uint32_t acquisition_time;     /**< Acquisition time in microseconds. */
    int32_t vref_mv;               /**< Reference voltage in millivolts. */
};

int adc_init(const struct adc_config *cfg);
int adc_read_raw(const struct adc_config *cfg, int16_t *raw_val);
float adc_read_normalized(const struct adc_config *cfg);
int adc_read_voltage(const struct adc_config *cfg, int32_t *out_mv);

#endif // ADC_H
