/**
 * @file brightness_thread.h
 * @brief Brightness measurement thread interface.
 *
 * This module defines the interface for a Zephyr thread that performs
 * ambient light measurement using a phototransistor connected to an ADC.
 * 
 * The thread waits for measurement requests triggered by the main
 * application and updates a shared brightness value in percentage.
 */

#ifndef BRIGHTNESS_THREAD_H
#define BRIGHTNESS_THREAD_H

#include <zephyr/kernel.h>
#include "sensors/adc/adc.h"

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
    /** Semaphore to trigger a new brightness measurement. */
    struct k_sem measure_request;
};

/**
 * @brief Initializes and starts the brightness measurement thread.
 *
 * This function sets up synchronization primitives and creates a Zephyr
 * thread that waits for measurement requests. When a request is received,
 * the thread performs an ADC reading, converts the result into a
 * percentage (0–100%), and updates the context.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 */
void start_brightness_thread(struct system_context *ctx);

/**
 * @brief Requests a brightness measurement.
 *
 * This function can be called from the main thread or any other thread.
 * It signals the brightness thread to perform a new measurement.
 *
 * @param ctx Pointer to the system context associated with the thread.
 */
void request_brightness_measurement(struct system_context *ctx);

#endif /* BRIGHTNESS_THREAD_H */
