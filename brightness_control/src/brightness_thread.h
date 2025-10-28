/**
 * @file brightness_thread.h
 * @brief Brightness measurement thread interface.
 *
 * This module defines the interface for a Zephyr thread that performs
 * ambient light measurement using a phototransistor connected to an ADC.
 * 
 * The thread periodically measures brightness only when the system mode
 * is set to NORMAL. Other modes stop measurement activity.
 */

#ifndef BRIGHTNESS_THREAD_H
#define BRIGHTNESS_THREAD_H

#include "main.h"

/**
 * @brief Initializes and starts the brightness measurement thread.
 *
 * This function sets up synchronization primitives and creates a Zephyr
 * thread that periodically measures brightness when the system mode is NORMAL.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 */
void start_brightness_thread(struct system_context *ctx);

#endif /* BRIGHTNESS_THREAD_H */
