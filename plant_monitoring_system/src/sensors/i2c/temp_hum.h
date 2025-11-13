/**
 * @file temp_hum.h
 * @brief Interface for the Si7021 temperature and humidity sensor using Zephyr I2C API.
 *
 * Provides functions to initialize the sensor and read temperature (°C) and
 * relative humidity (%RH) via I2C.
 */

#ifndef TEMP_HUM_H
#define TEMP_HUM_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* === Si7021 I2C Configuration === */
#define TEMP_HUM_I2C_ADDR          0x40  /**< Default I2C address of Si7021 */

/* === Si7021 command set === */
#define SI7021_MEAS_RH_HOLD        0xE5  /**< Measure Relative Humidity, Hold Master mode */
#define SI7021_MEAS_TEMP_HOLD      0xE3  /**< Measure Temperature, Hold Master mode */
#define SI7021_READ_TEMP_FROM_RH   0xE0  /**< Read Temperature from previous RH measurement */
#define SI7021_RESET               0xFE  /**< Soft reset command */

/* === Function Prototypes === */

/**
 * @brief Initialize the Si7021 temperature and humidity sensor.
 *
 * Checks if the I2C device is ready and performs a soft reset.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_init(const struct i2c_dt_spec *dev);

/**
 * @brief Read temperature in degrees Celsius from the sensor.
 *
 * Performs a measurement and converts the raw data to °C.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param temperature Pointer to store the temperature in °C.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_temperature(const struct i2c_dt_spec *dev, float *temperature);

/**
 * @brief Read relative humidity in percent from the sensor.
 *
 * Performs a measurement and converts the raw data to %RH.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param humidity Pointer to store the relative humidity in %RH.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_humidity(const struct i2c_dt_spec *dev, float *humidity);

#endif /* TEMP_HUM_H */
