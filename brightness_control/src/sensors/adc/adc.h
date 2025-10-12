#ifndef ADC_H
#define ADC_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

/**
 * @brief Initialize the ADC.
 *
 * @param dev Pointer to the ADC device.
 * @param cfg Pointer to the ADC channel configuration structure.
 * @return 0 on success, negative error code on failure.
 */
int adc_init(const struct device *dev, const struct adc_channel_cfg *cfg);

/**
 * @brief Read the raw ADC value.
 *
 * @param dev Pointer to the ADC device.
 * @param raw_val Pointer to store raw ADC sample.
 * @param resolution ADC resolution (bits).
 * @param channel_id ADC channel ID.
 * @param buffer Pointer to buffer to store sample.
 * @param buffer_size Buffer size.
 * @return 0 on success, negative error code on failure.
 */
int adc_read_raw(const struct device *dev,
                             int16_t *raw_val,
                             int resolution,
                             int channel_id,
                             int16_t *buffer,
                             size_t buffer_size);

/**
 * @brief Read normalized ADC value between 0.0 and 1.0.
 *
 * @param dev Pointer to the ADC device.
 * @param resolution ADC resolution (bits).
 * @param channel_id ADC channel ID.
 * @param buffer Pointer to buffer to store sample.
 * @param buffer_size Buffer size.
 * @return Normalized value (0–1), or -1.0 on failure.
 */
float adc_read_normalized(const struct device *dev,
                                      int resolution,
                                      int channel_id,
                                      int16_t *buffer,
                                      size_t buffer_size);

/**
 * @brief Read ADC value in millivolts.
 *
 * @param dev Pointer to the ADC device.
 * @param cfg Pointer to ADC channel configuration.
 * @param out_mv Pointer to store result in millivolts.
 * @param resolution ADC resolution (bits).
 * @param channel_id ADC channel ID.
 * @param buffer Pointer to buffer to store sample.
 * @param buffer_size Buffer size.
 * @return 0 on success, negative error code on failure.
 */
int adc_read_voltage(const struct device *dev,
                                 const struct adc_channel_cfg *cfg,
                                 int32_t *out_mv,
                                 int resolution,
                                 int channel_id,
                                 int16_t *buffer,
                                 size_t buffer_size);

#endif // ADC_H
