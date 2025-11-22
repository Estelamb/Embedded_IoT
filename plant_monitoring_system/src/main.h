/**
 * @file main.h
 * @brief Shared definitions for the Plant Monitoring System.
 *
 * This header defines shared data structures and enumerations
 * used across the main application, sensors thread, and GPS thread.
 * It provides a unified context for system configuration and
 * sensor measurements.
 */

#ifndef MAIN_H
#define MAIN_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "sensors/adc/adc.h"
#include "sensors/i2c/i2c.h"
#include "sensors/i2c/accel.h"
#include "sensors/i2c/temp_hum.h"
#include "sensors/i2c/color.h"
#include "sensors/gps/gps.h"
#include "sensors/led/rgb_led.h"
#include "sensors/led/board_led.h"
#include "sensors/user_button/user_button.h"

/** @brief Measurement interval in TEST mode (milliseconds). */
#define TEST_MODE_CADENCE 2000

/** @brief Measurement interval in NORMAL mode (milliseconds). */
#define NORMAL_MODE_CADENCE 30000

/**
 * @enum system_mode_t
 * @brief System operating modes.
 *
 * These modes define how the system behaves:
 * - **TEST_MODE:** Shows the dominant detected color through the RGB LED.
 * - **NORMAL_MODE:** Periodically measures sensors and alerts if any reading
 *   is out of range.
 * - **ADVANCED_MODE:** Operates silently with minimal LED feedback.
 */
typedef enum {
    TEST_MODE = 0,    /**< Test mode – displays the dominant color. */
    NORMAL_MODE,      /**< Normal mode – periodic measurement and alerts. */
    ADVANCED_MODE     /**< Advanced mode – minimal visual feedback. */
} system_mode_t;

/**
 * @struct system_context
 * @brief Shared system context between main, sensors, and GPS threads.
 *
 * This structure contains pointers to configuration objects, semaphores,
 * and shared state used to coordinate between the main, sensors, and GPS threads.
 */
struct system_context {
    struct adc_config *phototransistor; /**< Phototransistor ADC configuration. */
    struct adc_config *soil_moisture;   /**< Soil moisture ADC configuration. */

    struct i2c_dt_spec *accelerometer;  /**< Accelerometer I2C device specification. */
    uint8_t accel_range;                /**< Accelerometer full-scale range (e.g., 2G, 4G, 8G). */

    struct i2c_dt_spec *temp_hum;       /**< Temperature and humidity sensor I2C specification. */
    struct i2c_dt_spec *color;          /**< Color sensor I2C device specification. */
    struct gps_config *gps;             /**< GPS module configuration. */

    struct k_sem *main_sensors_sem;     /**< Semaphore for main-to-sensors synchronization. */
    struct k_sem *main_gps_sem;         /**< Semaphore for main-to-GPS synchronization. */
    struct k_sem *sensors_sem;          /**< Semaphore to trigger sensor measurement. */
    struct k_sem *gps_sem;              /**< Semaphore to trigger GPS measurement. */
};

/**
 * @struct system_measurement
 * @brief Shared sensor data between main, sensors, and GPS threads.
 *
 * Contains the most recent measurements for all sensors, stored
 * in atomic variables for thread-safe access.
 */
struct system_measurement {
    atomic_t brightness;  /**< Latest ambient brightness (0–100%). */
    atomic_t moisture;    /**< Latest soil moisture (0–100%). */

    atomic_t accel_x_g;   /**< Latest X-axis acceleration (in g). */
    atomic_t accel_y_g;   /**< Latest Y-axis acceleration (in g). */
    atomic_t accel_z_g;   /**< Latest Z-axis acceleration (in g). */

    atomic_t temp;        /**< Latest temperature (°C). */
    atomic_t hum;         /**< Latest relative humidity (%RH). */

    atomic_t red;         /**< Latest red color value (raw). */
    atomic_t green;       /**< Latest green color value (raw). */
    atomic_t blue;        /**< Latest blue color value (raw). */
    atomic_t clear;       /**< Latest clear color channel value (raw). */

    atomic_t gps_lat;     /**< Latest GPS latitude (degrees). */
    atomic_t gps_lon;     /**< Latest GPS longitude (degrees). */
    atomic_t gps_alt;     /**< Latest GPS altitude (meters). */
    atomic_t gps_sats;    /**< Latest number of satellites in view. */
    atomic_t gps_time;    /**< Latest GPS timestamp (float or encoded). */
};

#endif /* MAIN_H */
