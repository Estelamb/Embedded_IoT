#include "accel.h"
#include "i2c.h"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

int accel_set_range(const struct i2c_dt_spec *dev, uint8_t range) {
    return i2c_write_reg(dev, ACCEL_REG_XYZ_DATA_CFG, range & 0x03);
}

int accel_set_standby(const struct i2c_dt_spec *dev)
{
    uint8_t ctrl1;
    int ret = i2c_read_regs(dev, ACCEL_REG_CTRL1, &ctrl1, 1);
    if (ret < 0) return ret;

    ctrl1 &= ~0x01; // ACTIVE = 0
    return i2c_write_reg(dev, ACCEL_REG_CTRL1, ctrl1);
}

int accel_set_active(const struct i2c_dt_spec *dev)
{
    uint8_t ctrl1;
    int ret = i2c_read_regs(dev, ACCEL_REG_CTRL1, &ctrl1, 1);
    if (ret < 0) return ret;

    ctrl1 |= 0x01; // ACTIVE = 1
    return i2c_write_reg(dev, ACCEL_REG_CTRL1, ctrl1);
}

int accel_init(const struct i2c_dt_spec *dev, uint8_t range) {
    printk("Initializing ACCEL...\n");

    int ret = i2c_dev_ready(dev);
    if (ret < 0) return ret;

    uint8_t whoami;
    ret = i2c_read_regs(dev, ACCEL_REG_WHO_AM_I, &whoami, 1);
    if (ret < 0 || whoami != ACCEL_WHO_AM_I_VALUE) {
        printk("ACCEL WHO_AM_I mismatch: 0x%02X\n", whoami);
        return -EIO;
    }

    printk("ACCEL detected at 0x%02X\n", dev->addr);

    if (accel_set_standby(dev) < 0) {
        printk("Failed to set ACCEL to Standby mode\n");
        return -EIO;
    }

    if (accel_set_range(dev, range) < 0) {
        printk("Failed to set range to %d\n", range);
        return -EIO;
    }

    // From Standby mode to Active mode
    return accel_set_active(dev);
}

int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    int ret = i2c_read_regs(dev, ACCEL_REG_OUT_X_MSB, buf, 6);
    if (ret < 0) return ret;

    *x = (int16_t)(((int16_t)((buf[0] << 8) | buf[1])) >> 2);
    *y = (int16_t)(((int16_t)((buf[2] << 8) | buf[3])) >> 2);
    *z = (int16_t)(((int16_t)((buf[4] << 8) | buf[5])) >> 2);

    printk("Read ACCEL XYZ raw: X=%d, Y=%d, Z=%d\n", *x, *y, *z);
    return 0;
}


void accel_convert_to_g(int16_t raw, uint8_t range, float *g_value) {
    float sensitivity = (range == ACCEL_2G) ? 4096.0f :
                        (range == ACCEL_4G) ? 2048.0f : 1024.0f;
    *g_value = (float)raw / sensitivity;
}


void accel_convert_to_ms2(int16_t raw, uint8_t range, float *ms2_value) {
    float sensitivity = (range == ACCEL_2G) ? 4096.0f :
                        (range == ACCEL_4G) ? 2048.0f : 1024.0f;
    float g_value = (float)raw / sensitivity;
    *ms2_value = g_value * 9.80665f;
}
