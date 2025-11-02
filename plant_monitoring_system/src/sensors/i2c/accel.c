#include "accel.h"
#include "i2c.h"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

int accel_init(const struct i2c_dt_spec *dev) {
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

    // Activate device
    uint8_t ctrl1 = 0x01;
    return i2c_write_reg(dev, ACCEL_REG_CTRL_REG1, ctrl1);
}

int accel_set_range(const struct i2c_dt_spec *dev, uint8_t range) {
    return i2c_write_reg(dev, ACCEL_REG_XYZ_DATA_CFG, range & 0x03);
}

int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    int ret = i2c_read_regs(dev, ACCEL_REG_OUT_X_MSB, buf, 6);
    if (ret < 0) return ret;

    *x = (int16_t)((buf[0] << 8 | buf[1]) >> 2);
    *y = (int16_t)((buf[2] << 8 | buf[3]) >> 2);
    *z = (int16_t)((buf[4] << 8 | buf[5]) >> 2);
    return 0;
}

void accel_convert_to_g(int16_t raw, uint8_t range, float *g_value) {
    float sensitivity = (range == ACCEL_FS_2G) ? 4096.0f :
                        (range == ACCEL_FS_4G) ? 2048.0f : 1024.0f;
    *g_value = (float)raw / sensitivity;
}
