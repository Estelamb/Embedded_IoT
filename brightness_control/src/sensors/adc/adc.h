/**
 * @file adc.h
 * @brief Interface for ADC initialization and sampling using Zephyr drivers.
 */

#ifndef ADC_H
#define ADC_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define BUFFER_SIZE 1 /**< ADC sample buffer size (1 sample). */

/**
 * @brief ADC configuration structure.
 */
struct adc_config {
    const struct device *dev;      /**< Pointer to ADC device. */
    uint8_t channel_id;            /**< ADC channel number. */
    uint8_t resolution;            /**< ADC resolution in bits. */
    enum adc_gain gain;            /**< ADC gain setting. */
    enum adc_reference ref;        /**< ADC reference source. */
    uint32_t acquisition_time;     /**< Acquisition time in microseconds. */
    int32_t vref_mv;               /**< Reference voltage in millivolts. */
};

int adc_init(const struct adc_config *cfg);
int adc_read_raw(int16_t *raw_val);
float adc_read_normalized(void);
int adc_read_voltage(int32_t *out_mv);

#endif // ADC_H
