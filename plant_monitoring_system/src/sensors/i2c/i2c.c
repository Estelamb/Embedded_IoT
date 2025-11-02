#include "i2c.h"
#include <zephyr/sys/printk.h>

int i2c_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_write_read_dt(dev, &reg, 1, buf, len);
}

int i2c_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg, val };
    return i2c_write_dt(dev, data, sizeof(data));
}

int i2c_dev_ready(const struct i2c_dt_spec *dev) {
    if (!i2c_is_ready_dt(dev)) {
        printk("I2C device at address 0x%02X not ready\n", dev->addr);
        return -ENODEV;
    }
    return 0;
}
