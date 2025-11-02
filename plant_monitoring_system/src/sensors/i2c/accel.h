#ifndef ACCEL_H
#define ACCEL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#define ACCEL_I2C_ADDR     0x1D

/* Register definitions */
#define ACCEL_REG_WHO_AM_I         0x0D
#define ACCEL_REG_CTRL_REG1        0x2A
#define ACCEL_REG_XYZ_DATA_CFG     0x0E
#define ACCEL_REG_OUT_X_MSB        0x01
#define ACCEL_WHO_AM_I_VALUE       0x1A

/* Measurement range */
#define ACCEL_FS_2G  0x00
#define ACCEL_FS_4G  0x01
#define ACCEL_FS_8G  0x02

/* Function prototypes */
int accel_init(const struct i2c_dt_spec *dev);
int accel_set_range(const struct i2c_dt_spec *dev, uint8_t range);
int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z);
void accel_convert_to_g(int16_t raw_val, uint8_t range, float *g_out);

#endif // ACCEL_H
