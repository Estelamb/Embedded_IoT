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
    struct adc_config *phototransistor;   /**< Phototransistor ADC configuration */
    atomic_t brightness;                  /**< Latest brightness percent (0-100, atomic) */
    struct adc_config *soil_moisture;     /**< Soil moisture ADC configuration */
    atomic_t moisture;                    /**< Latest soil moisture percent (0-100, atomic) */
    struct k_sem *sensors_sem;            /**< Semaphore for sensors measurement */
    atomic_t mode;                        /**< Current operating mode (atomic enum) */
};

#endif /* MAIN_H */
