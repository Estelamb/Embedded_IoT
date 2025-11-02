/**
 * @file sensors_thread.h
 * @brief Interface for the sensors measurement thread.
 *
 * This module defines the interface for a Zephyr thread that performs
 * ambient light and soil moisture measurement using ADC sensors.
 *
 * The thread periodically measures both brightness and moisture only when
 * the system mode is set to NORMAL. Other modes suspend measurement activity.
 */

#ifndef SENSORS_THREAD_H
#define SENSORS_THREAD_H

#include "main.h"

/**
 * @brief Initializes and starts the sensors measurement thread.
 *
 * This function sets up synchronization primitives and creates a Zephyr
 * thread that periodically measures brightness and soil moisture when the
 * system mode is NORMAL.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 */
void start_sensors_thread(struct system_context *ctx);

#endif /* SENSORS_THREAD_H */
