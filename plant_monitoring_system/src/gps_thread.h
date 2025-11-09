/**
 * @file gps_thread.h
 * @brief Interface for the gps measurement thread.
 *
 * This module defines the interface for a Zephyr thread that performs
 * GPS data acquisition and processing.
 *
 * The thread periodically measures GPS data only when the system mode
 * is set to NORMAL. Other modes suspend measurement activity.
 */

#ifndef GPS_THREAD_H
#define GPS_THREAD_H

#include "main.h"

/**
 * @brief Initializes and starts the gps measurement thread.
 *
 * This function sets up synchronization primitives and creates a Zephyr
 * thread that periodically measures GPS data when the system mode is
 * NORMAL.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 * @param measure Pointer to a valid @ref system_measurement structure.
 */
void start_gps_thread(struct system_context *ctx, struct system_measurement *measure);

#endif /* GPS_THREAD_H */
