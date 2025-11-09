/**
 * @file gps_thread.c
 * @brief Implementation of the gps measurement thread.
 *
 * The gps thread runs continuously, checking the current
 * operating mode stored in the shared context. When the mode is NORMAL,
 * it periodically reads GPS data.
 */

#include "gps_thread.h"
#include "gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define GPS_THREAD_STACK_SIZE 1024
#define GPS_THREAD_PRIORITY 5
#define GPS_MEASURE_INTERVAL_MS 2000 /**< Measurement interval in NORMAL mode. */

/** Stack allocation for the gps thread. */
K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);

/** Thread control block. */
static struct k_thread gps_thread_data;

/* Timer to trigger periodic gps measurements */
static struct k_timer gps_timer;

/* Semaphore to wake up the thread when the timer expires */
static struct k_sem gps_timer_sem;

/**
 * @brief Timer handler: Gives the semaphore when the timer expires.
 */
static void gps_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&gps_timer_sem);
}

/**
 * @brief GPS measurement thread function.
 *
 * Periodically checks the current operating mode. When the system is
 * in NORMAL mode, this thread measures sensor values, converts them
 * to percentages, and updates context variables atomically.
 *
 * @param arg1 Pointer to a @ref system_context structure.
 * @param arg2 Pointer to a @ref system_measurement structure.
 * @param arg3 Unused.
 */
static void gps_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx = (struct system_context *)arg1;
    struct system_measurement *measure = (struct system_measurement *)arg2;
    system_mode_t previous_mode = atomic_get(&ctx->mode);
    system_mode_t actual_mode = previous_mode;

    gps_data_t data;

    if (actual_mode == NORMAL_MODE) {
        k_timer_start(&gps_timer, K_NO_WAIT, K_MSEC(GPS_MEASURE_INTERVAL_MS));
    }

    while (1) {
        actual_mode = atomic_get(&ctx->mode);

        if (actual_mode == NORMAL_MODE) {
            if (previous_mode != NORMAL_MODE) {
                k_timer_start(&gps_timer, K_NO_WAIT, K_MSEC(GPS_MEASURE_INTERVAL_MS));
            }

            previous_mode = NORMAL_MODE;

            /* Read GPS */
            if (gps_get_data(&data)) {
                printk("GPS: %.6f° %.6f° Alt: %.1f m Sats: %d HDOP: %.1f\n",
                       data.latitude, data.longitude, data.altitude,
                       data.satellites, data.hdop);
                atomic_set(measure->gps_lat, *(atomic_t *)&data.latitude);
                atomic_set(measure->gps_lon, *(atomic_t *)&data.longitude);
                atomic_set(measure->gps_alt, *(atomic_t *)&data.altitude);
                atomic_set(measure->gps_sats, *(atomic_t *)&data.satellites);
                atomic_set(measure->gps_hdop, *(atomic_t *)&data.hdop);
            }

            /* Wait for next measurement */
            k_sem_take(&gps_timer_sem, K_FOREVER);
        } else {
            if (previous_mode == NORMAL_MODE) {
                k_timer_stop(&gps_timer);
            }

            previous_mode = actual_mode;
            k_sem_take(ctx->gps_sem, K_FOREVER);
        }
    }
}

/**
 * @brief Starts the gps measurement thread.
 *
 * This function initializes core components and starts the thread
 * which will run the gps polling loop.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 * @param measure Pointer to a valid @ref system_measurement structure.
 */
void start_gps_thread(struct system_context *ctx, struct system_measurement *measure) {
    /* Init timer and measurement semaphore */
    k_sem_init(&gps_timer_sem, 0, 1);
    k_timer_init(&gps_timer, gps_timer_handler, NULL);

    /* Start thread */
    k_thread_create(&gps_thread_data,
                    gps_stack,
                    K_THREAD_STACK_SIZEOF(gps_stack),
                    gps_thread_fn,
                    ctx, measure, NULL,
                    GPS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&gps_thread_data, "gps_thread");
}
