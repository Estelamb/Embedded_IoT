#ifndef ADC_H
#define ADC_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define BUFFER_SIZE 1

struct adc_config {
    const struct device *dev;
    uint8_t channel_id;
    uint8_t resolution;
    enum adc_gain gain;
    enum adc_reference ref;
    uint32_t acquisition_time;
    int32_t vref_mv; // por ejemplo, 3300 mV
};

/**
 * @brief Inicializa el ADC con la configuración indicada.
 *
 * @param cfg Puntero a la configuración del ADC.
 * @return 0 si tuvo éxito, o un código de error.
 */
int adc_init(const struct adc_config *cfg);

/**
 * @brief Lee el valor RAW del ADC.
 *
 * @param raw_val Puntero donde se almacenará la lectura RAW.
 * @return 0 si tuvo éxito, o un código de error.
 */
int adc_read_raw(int16_t *raw_val);

/**
 * @brief Lee el valor normalizado (0.0 a 1.0).
 *
 * @return Valor normalizado, o -1.0 en caso de error.
 */
float adc_read_normalized(void);

/**
 * @brief Lee el voltaje en milivoltios.
 *
 * @param out_mv Puntero donde se almacenará el voltaje (mV).
 * @return 0 si tuvo éxito, o un código de error.
 */
int adc_read_voltage(int32_t *out_mv);


#endif // ADC_H
