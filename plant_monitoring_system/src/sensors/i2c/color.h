/**
 * @file color.h
 * @brief Interface for the TCS34725 color sensor using Zephyr I2C API.
 *
 * Provides functions to initialize, configure, and read RGB values
 * from the TCS34725 color sensor via I2C.
 */

#ifndef COLOR_H
#define COLOR_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* === TCS34725 I2C Configuration === */
#define COLOR_I2C_ADDR    0x29  /**< I2C address of TCS34725 */
#define COLOR_COMMAND 0x80  /**< Command bit */
#define AUTO_INCREMENT 0x20 /**< Enable address auto-increment */

/* === Register Addresses === */
#define COLOR_ENABLE   0x00
#define COLOR_ATIME    0x01
#define COLOR_CONTROL  0x0F
#define COLOR_CLEAR_L  0x14
#define COLOR_RED_L    0x16
#define COLOR_GREEN_L  0x18
#define COLOR_BLUE_L   0x1A

/* === ENABLE register bits === */
#define ENABLE_PON  0x01  /**< Power ON */
#define ENABLE_AEN  0x02  /**< ADC Enable */

/* === Gain settings === */
#define GAIN_1X    0x00
#define GAIN_4X    0x01
#define GAIN_16X   0x02
#define GAIN_60X   0x03

/* === Integration time settings (ATIME register) === */
#define INTEGRATION_2_4MS   0xFF
#define INTEGRATION_24MS    0xF6
#define INTEGRATION_101MS   0xD5
#define INTEGRATION_154MS   0xC0
#define INTEGRATION_700MS   0x00

/** @brief Structure for color sensor data */
typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} ColorSensorData;


/* === Function prototypes === */
int color_init(const struct i2c_dt_spec *dev);
int color_wake_up(const struct i2c_dt_spec *dev);
int color_sleep(const struct i2c_dt_spec *dev);
int color_set_gain(const struct i2c_dt_spec *dev, uint8_t gain);
int color_set_integration(const struct i2c_dt_spec *dev, uint8_t atime);
int color_read_rgb(const struct i2c_dt_spec *dev, ColorSensorData *data);

#endif /* COLOR_H */
