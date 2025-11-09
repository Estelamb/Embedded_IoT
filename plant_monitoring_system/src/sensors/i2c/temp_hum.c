#include "temp_hum.h"
#include "i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/**
 * @brief Write a single command to the Si7021.
 */
static int si7021_write_cmd(const struct i2c_dt_spec *dev, uint8_t cmd)
{
    return i2c_write_dt(dev, &cmd, 1);
}

/**
 * @brief Read data from Si7021 after sending a command.
 */
static int si7021_read_data(const struct i2c_dt_spec *dev, uint8_t cmd, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(dev, &cmd, 1, buf, len);
}

/**
 * @brief Initialize the Si7021 sensor.
 */
int temp_hum_init(const struct i2c_dt_spec *dev)
{
    if (!device_is_ready(dev->bus)) {
        printk("[TEMP_HUM] I2C bus not ready\n");
        return -ENODEV;
    }

    printk("[TEMP_HUM] Initializing Si7021...\n");
    int ret = si7021_write_cmd(dev, SI7021_RESET);
    if (ret < 0) {
        printk("[TEMP_HUM] Reset failed (%d)\n", ret);
        return ret;
    }

    k_msleep(50); // Give sensor time to reset

    printk("[TEMP_HUM] Si7021 initialized successfully\n");
    return 0;
}

/**
 * @brief Read relative humidity in %RH.
 */
int temp_hum_read_humidity(const struct i2c_dt_spec *dev, float *humidity)
{
    uint8_t buf[2];
    int ret = si7021_read_data(dev, SI7021_MEAS_RH_HOLD, buf, sizeof(buf));
    if (ret < 0) {
        printk("[TEMP_HUM] Failed to read humidity (%d)\n", ret);
        return ret;
    }

    uint16_t raw_rh = ((uint16_t)buf[0] << 8) | buf[1];
    float rh = ((125.0f * raw_rh) / 65536.0f) - 6.0f;

    if (rh < 0.0f) rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;

    *humidity = rh;
    return 0;
}

/**
 * @brief Read temperature in °C.
 */
int temp_hum_read_temperature(const struct i2c_dt_spec *dev, float *temperature)
{
    uint8_t buf[2];
    int ret = si7021_read_data(dev, SI7021_MEAS_TEMP_HOLD, buf, sizeof(buf));
    if (ret < 0) {
        printk("[TEMP_HUM] Failed to read temperature (%d)\n", ret);
        return ret;
    }

    uint16_t raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
    float temp = ((175.72f * raw_temp) / 65536.0f) - 46.85f;

    *temperature = temp;
    return 0;
}
