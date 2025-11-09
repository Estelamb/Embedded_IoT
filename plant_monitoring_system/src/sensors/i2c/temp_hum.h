#ifndef TEMP_HUM_H
#define TEMP_HUM_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* === Si7021 I2C Configuration === */
#define TEMP_HUM_I2C_ADDR          0x40  /**< Si7021 default I2C address */

/* Si7021 command set */
#define SI7021_MEAS_RH_HOLD        0xE5  /**< Measure Relative Humidity, Hold Master */
#define SI7021_MEAS_TEMP_HOLD      0xE3  /**< Measure Temperature, Hold Master */
#define SI7021_READ_TEMP_FROM_RH   0xE0  /**< Read Temperature from previous RH measurement */
#define SI7021_RESET               0xFE  /**< Soft reset command */

/* === Function Prototypes === */

/**
 * @brief Initialize the Si7021 temperature/humidity sensor.
 *
 * @param dev Pointer to a valid I2C device specification (typically i2c2).
 * @return 0 on success, negative error code otherwise.
 */
int temp_hum_init(const struct i2c_dt_spec *dev);

/**
 * @brief Read temperature from the Si7021.
 *
 * @param dev Pointer to a valid I2C device specification.
 * @param temperature Pointer to store the temperature in °C.
 * @return 0 on success, negative error code otherwise.
 */
int temp_hum_read_temperature(const struct i2c_dt_spec *dev, float *temperature);

/**
 * @brief Read relative humidity from the Si7021.
 *
 * @param dev Pointer to a valid I2C device specification.
 * @param humidity Pointer to store the relative humidity in %RH.
 * @return 0 on success, negative error code otherwise.
 */
int temp_hum_read_humidity(const struct i2c_dt_spec *dev, float *humidity);

#endif /* TEMP_HUM_H */
