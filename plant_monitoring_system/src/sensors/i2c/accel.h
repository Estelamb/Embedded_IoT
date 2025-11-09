#ifndef ACCEL_H
#define ACCEL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* Device address */
#define ACCEL_I2C_ADDR          0x1D
#define ACCEL_REG_WHO_AM_I      0x0D
#define ACCEL_WHO_AM_I_VALUE    0x1A /* Expected value */

/* Power control modes */
#define ACCEL_REG_CTRL1         0x2A
#define ACCEL_REG_CTRL2         0x2B

/* Measurement range */
#define ACCEL_REG_XYZ_DATA_CFG  0x0E /* Sets the range */
#define ACCEL_2G                0x00
#define ACCEL_4G                0x01
#define ACCEL_8G                0x02

/* Output register addresses */
#define ACCEL_REG_OUT_X_MSB     0x01
#define ACCEL_REG_OUT_X_LSB     0x02
#define ACCEL_REG_OUT_Y_MSB     0x03
#define ACCEL_REG_OUT_Y_LSB     0x04
#define ACCEL_REG_OUT_Z_MSB     0x05
#define ACCEL_REG_OUT_Z_LSB     0x06

/* Function prototypes */
int accel_init(const struct i2c_dt_spec *dev, uint8_t range);
int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z);
void accel_convert_to_g(int16_t raw_val, uint8_t range, float *g_out);
void accel_convert_to_ms2(int16_t raw, uint8_t range, float *ms2_value);

#endif // ACCEL_H
