/**
 * @file main.h
 * @brief Shared definitions for the brightness control system.
 *
 * This header defines the system mode enumeration and shared context
 * structure used by both the main application and the brightness thread.
 */

#ifndef MAIN_H
#define MAIN_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "sensors/adc/adc.h"
#include "sensors/i2c/i2c.h"
#include "sensors/i2c/accel.h"
#include "sensors/rgb_led/rgb_led.h"
#include "sensors/user_button/user_button.h"

/**
 * @brief System operating modes.
 *
 * - OFF_MODE: Device is turned off and LEDs are off.
 * - NORMAL_MODE: Normal operation where ambient brightness is measured
 *   and the LED color indicates the brightness level.
 * - BLUE_MODE: Special mode where the blue LED is shown continuously.
 */
typedef enum {
    OFF_MODE = 0,
    NORMAL_MODE,
    BLUE_MODE
} system_mode_t;

/**
 * @brief Shared system context between main and sensors thread.
 */
struct system_context {
    struct adc_config *phototransistor; /**< Phototransistor ADC configuration */
    atomic_t brightness;                /**< Latest brightness percent (0-100, atomic) */
    
    struct adc_config *soil_moisture;   /**< Soil moisture ADC configuration */
    atomic_t moisture;                  /**< Latest soil moisture percent (0-100, atomic) */

    struct i2c_dt_spec *accelerometer;  /**< Accelerometer I2C device specification */
    uint8_t accel_range;                /**< Full-scale range (FS_2G, FS_4G, FS_8G) */
    atomic_t accel_x_g;                 /**< Latest X-axis acceleration in g (atomic float) */
    atomic_t accel_y_g;                 /**< Latest Y-axis acceleration in g (atomic float) */
    atomic_t accel_z_g;                 /**< Latest Z-axis acceleration in g (atomic float) */

    struct k_sem *sensors_sem;          /**< Semaphore for sensors measurement */
    atomic_t mode;                      /**< Current operating mode (atomic enum) */
};

#endif /* MAIN_H */
