#ifndef I2C_H
#define I2C_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/**
 * @brief Read multiple bytes from a device register over I2C.
 *
 * @param dev Pointer to the I2C device descriptor (from devicetree).
 * @param reg Register address to start reading.
 * @param buf Pointer to buffer to store the data.
 * @param len Number of bytes to read.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len);

/**
 * @brief Write a single byte to a device register over I2C.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param reg Register address to write to.
 * @param val Value to write.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val);

/**
 * @brief Check if a device is reachable on I2C bus.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 if ready, -ENODEV otherwise.
 */
int i2c_dev_ready(const struct i2c_dt_spec *dev);

#endif // I2C_H
