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
 * @brief Shared system context between main and brightness thread.
 */
struct system_context {
    /** Pointer to ADC configuration and device information. */
    struct adc_config *adc;
    /** Latest measured brightness value (0.0 - 100.0%). */
    float brightness;
    /** Mutex to protect shared data. */
    struct k_mutex lock;
    /** Current operating mode. */
    system_mode_t mode;
};

#endif /* MAIN_H */
