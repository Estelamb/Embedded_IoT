/**
 * @file color.c
 * @brief Implementation for TCS34725 color sensor driver on Zephyr.
 */

#include "color.h"
#include "i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* === Internal helper functions === */
static int color_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { COLOR_COMMAND | reg, val };
    return i2c_write_dt(dev, buf, sizeof(buf));
}

static int color_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t reg_cmd = COLOR_COMMAND | AUTO_INCREMENT | reg;
    return i2c_write_read_dt(dev, &reg_cmd, 1, buf, len);
}

/* === Public API === */

int color_init(const struct i2c_dt_spec *dev)
{
    if (!device_is_ready(dev->bus)) {
        printk("[COLOR SENSOR] - I2C bus not ready\n");
        return -ENODEV;
    }

    printk("[COLOR] Initializing TCS34725...\n");

    /* Power ON then enable ADC */
    if (color_wake_up(dev) < 0) {
        printk("[COLOR SENSOR] - Failed to wake up sensor\n");
        return -EIO;
    }

    /* Default settings */
    color_set_gain(dev, GAIN_4X);
    color_set_integration(dev, INTEGRATION_154MS);

    printk("[COLOR] TCS34725 initialized\n");
    return 0;
}

int color_wake_up(const struct i2c_dt_spec *dev)
{
    int ret = color_write_reg(dev, COLOR_ENABLE, ENABLE_PON);
    if (ret < 0) return ret;

    k_msleep(3); /* Wait power-on */

    ret = color_write_reg(dev, COLOR_ENABLE, ENABLE_PON | ENABLE_AEN);
    if (ret < 0) return ret;

    k_msleep(3);
    return 0;
}

int color_sleep(const struct i2c_dt_spec *dev)
{
    return color_write_reg(dev, COLOR_ENABLE, 0x00);
}

int color_set_gain(const struct i2c_dt_spec *dev, uint8_t gain)
{
    if (gain > GAIN_60X) gain = GAIN_1X;
    return color_write_reg(dev, COLOR_CONTROL, gain);
}

int color_set_integration(const struct i2c_dt_spec *dev, uint8_t atime)
{
    return color_write_reg(dev, COLOR_ATIME, atime);
}

int color_read_rgb(const struct i2c_dt_spec *dev, ColorSensorData *data)
{
    uint8_t buf[8];
    int ret = color_read_regs(dev, COLOR_CLEAR_L, buf, sizeof(buf));
    if (ret < 0) {
        printk("[COLOR SENSOR] - Failed to read RGB data (%d)\n", ret);
        return ret;
    }

    /* Combine low and high bytes */
    uint16_t clear = (buf[1] << 8) | buf[0];
    uint16_t red   = (buf[3] << 8) | buf[2];
    uint16_t green = (buf[5] << 8) | buf[4];
    uint16_t blue  = (buf[7] << 8) | buf[6];

    /* Avoid divide-by-zero */
    if (clear == 0) clear = 1;

    /* Store raw values */
    data->clear = clear;
    data->red   = red;
    data->green = green;
    data->blue  = blue;

    return 0;
}
