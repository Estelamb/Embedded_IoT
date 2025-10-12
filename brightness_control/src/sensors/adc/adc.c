#include "adc.h"
#include <zephyr/sys/printk.h>

int adc_init(const struct device *dev, const struct adc_channel_cfg *cfg) {
    if (!device_is_ready(dev)) {
        printk("Error: ADC device not ready\n");
        return -ENODEV;
    }

    int ret = adc_channel_setup(dev, cfg);
    if (ret < 0) {
        printk("Error: Failed to setup ADC channel (%d)\n", ret);
        return ret;
    }

    printk("ADC initialized successfully\n");
    return 0;
}

int adc_read_raw(const struct device *dev,
                             int16_t *raw_val,
                             int resolution,
                             int channel_id,
                             int16_t *buffer,
                             size_t buffer_size) {
    if (!device_is_ready(dev)) {
        printk("Error: ADC device not ready\n");
        return -ENODEV;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(channel_id),
        .buffer = buffer,
        .buffer_size = buffer_size * sizeof(int16_t),
        .resolution = resolution,
    };

    int ret = adc_read(dev, &sequence);
    if (ret < 0) {
        printk("Error: ADC read failed (%d)\n", ret);
        return ret;
    }

    *raw_val = buffer[0];
    return 0;
}

float adc_read_normalized(const struct device *dev,
                                      int resolution,
                                      int channel_id,
                                      int16_t *buffer,
                                      size_t buffer_size) {
    int16_t raw = 0;
    if (adc_read_raw(dev, &raw, resolution, channel_id, buffer, buffer_size) != 0) {
        return -1.0f;
    }

    return (float)raw / ((1 << resolution) - 1);
}

int adc_read_voltage(const struct device *dev,
                                 const struct adc_channel_cfg *cfg,
                                 int32_t *out_mv,
                                 int resolution,
                                 int channel_id,
                                 int16_t *buffer,
                                 size_t buffer_size) {
    int16_t raw;
    int ret = adc_read_raw(dev, &raw, resolution, channel_id, buffer, buffer_size);
    if (ret < 0) {
        return ret;
    }

    *out_mv = raw;
    return adc_raw_to_millivolts(adc_ref_internal(dev),
                                 cfg->gain,
                                 resolution,
                                 out_mv);
}
