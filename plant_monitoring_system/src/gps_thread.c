/**
 * @file gps_thread.c
 * @brief Implementation of the GPS measurement thread.
 *
 * This module defines the GPS measurement thread responsible for
 * periodically acquiring GPS data, parsing it, and updating the
 * shared measurement structure with scaled integer values.
 * 
 * ## Features:
 * - Periodic GPS polling controlled by system mode (TEST/NORMAL/ADVANCED)
 * - Thread synchronization through semaphores and poll events
 * - Scaled integer storage for latitude, longitude, and altitude
 */

#include "gps_thread.h"
#include "sensors/gps/gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* --- Thread configuration --------------------------------------------------- */
#define GPS_THREAD_STACK_SIZE 1024  /**< Stack size allocated for the GPS thread. */
#define GPS_THREAD_PRIORITY   5     /**< Thread priority (lower = higher priority). */

K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE); /**< GPS thread stack. */
static struct k_thread gps_thread_data;                  /**< GPS thread control block. */

/* --- Timer and synchronization ---------------------------------------------- */
static struct k_timer gps_timer;     /**< Periodic timer for GPS measurements. */
static struct k_sem gps_timer_sem;   /**< Semaphore released when timer expires. */

/** @brief Poll events for timer expiration and manual GPS trigger. */
static struct k_poll_event gps_poll_events[2];

/* ---------------------------------------------------------------------------
 * Helper functions
 * ---------------------------------------------------------------------------*/

/**
 * @brief Timer callback handler.
 *
 * Called automatically when the GPS timer expires. It wakes up the GPS
 * measurement thread by releasing the @ref gps_timer_sem semaphore.
 *
 * @param timer_id Pointer to the timer that triggered the event.
 */
static void gps_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&gps_timer_sem);
}

/**
 * @brief Initialize poll events used by the GPS thread.
 *
 * Sets up two poll events:
 * - Index 0 → Triggered by GPS timer expiration.
 * - Index 1 → Triggered by manual semaphore release (e.g., mode change).
 *
 * @param ctx Pointer to the shared system context.
 */
static void init_gps_poll_events(struct system_context *ctx) {
    gps_poll_events[0].type  = K_POLL_TYPE_SEM_AVAILABLE;
    gps_poll_events[0].mode  = K_POLL_MODE_NOTIFY_ONLY;
    gps_poll_events[0].sem   = &gps_timer_sem;
    gps_poll_events[0].state = K_POLL_STATE_NOT_READY;

    gps_poll_events[1].type  = K_POLL_TYPE_SEM_AVAILABLE;
    gps_poll_events[1].mode  = K_POLL_MODE_NOTIFY_ONLY;
    gps_poll_events[1].sem   = ctx->gps_sem;
    gps_poll_events[1].state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Wait for a GPS event (timer or manual trigger).
 *
 * Blocks the thread until either the GPS timer expires or an external
 * semaphore (ctx->gps_sem) is released. After handling the event,
 * it resets both poll states.
 */
static void wait_for_gps_event(void) {
    k_poll(gps_poll_events, 2, K_FOREVER);

    if (gps_poll_events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&gps_timer_sem, K_NO_WAIT);
    }
    if (gps_poll_events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(gps_poll_events[1].sem, K_NO_WAIT);
    }

    gps_poll_events[0].state = K_POLL_STATE_NOT_READY;
    gps_poll_events[1].state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Update GPS timer according to the current system mode.
 *
 * - **TEST_MODE:** Shorter interval for frequent updates.
 * - **NORMAL_MODE:** Standard measurement cadence.
 * - **ADVANCED_MODE:** Timer remains stopped.
 *
 * @param mode Current operating mode.
 */
static void update_gps_timer(system_mode_t mode) {
    k_timer_stop(&gps_timer);

    switch (mode) {
        case TEST_MODE:
            k_timer_start(&gps_timer, K_NO_WAIT, K_MSEC(TEST_MODE_CADENCE));
            break;
        case NORMAL_MODE:
            k_timer_start(&gps_timer, K_NO_WAIT, K_MSEC(NORMAL_MODE_CADENCE));
            break;
        default:
            /* ADVANCED or other modes: timer stays stopped */
            break;
    }
}

/**
 * @brief Read GPS data and update shared measurements.
 *
 * This function waits for a valid NMEA GGA sentence, parses its fields,
 * and updates the shared @ref system_measurement structure with scaled
 * integer values for safe atomic storage.
 *
 * @param data Pointer to a persistent @ref gps_data_t buffer.
 * @param measure Pointer to the shared measurement structure.
 * @param ctx Pointer to the shared system context.
 */
static void read_gps_data(gps_data_t *data,
                          struct system_measurement *measure,
                          struct system_context *ctx) {

    if (gps_wait_for_gga(data, K_MSEC(2000)) == 0) {
        /* Convert and store scaled GPS values */
        atomic_set(&measure->gps_lat,  (int32_t)(data->lat  * 1e6f));
        atomic_set(&measure->gps_lon,  (int32_t)(data->lon  * 1e6f));
        atomic_set(&measure->gps_alt,  (int32_t)(data->alt  * 100.0f));
        atomic_set(&measure->gps_sats, (int32_t)data->sats);

        /* Parse UTC time in HHMMSS format */
        if (strlen(data->utc_time) >= 6) {
            int hh = (data->utc_time[0] - '0') * 10 + (data->utc_time[1] - '0');
            int mm = (data->utc_time[2] - '0') * 10 + (data->utc_time[3] - '0');
            int ss = (data->utc_time[4] - '0') * 10 + (data->utc_time[5] - '0');

            int time_int = hh * 10000 + mm * 100 + ss; /**< Encoded time as HHMMSS integer. */
            atomic_set(&measure->gps_time, time_int);
        } else {
            atomic_set(&measure->gps_time, -1); /**< Invalid or missing time. */
        }

    } else {
        printk("[GPS] - Timeout or invalid data\n");
    }
}

/* ---------------------------------------------------------------------------
 * GPS Thread
 * ---------------------------------------------------------------------------*/

/**
 * @brief GPS measurement thread entry function.
 *
 * Continuously monitors the system mode and performs GPS readings
 * according to the configured update rate. Synchronizes with the
 * main thread via semaphores.
 *
 * @param arg1 Pointer to the shared @ref system_context structure.
 * @param arg2 Pointer to the shared @ref system_measurement structure.
 * @param arg3 Unused (set to NULL).
 */
static void gps_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx = (struct system_context *)arg1;
    struct system_measurement *measure = (struct system_measurement *)arg2;

    system_mode_t previous_mode = atomic_get(&ctx->mode);
    system_mode_t current_mode  = previous_mode;

    gps_data_t gps_data = {0};

    init_gps_poll_events(ctx);
    update_gps_timer(current_mode);

    while (1) {
        current_mode = atomic_get(&ctx->mode);

        /* Handle mode transitions */
        if (current_mode != previous_mode) {
            update_gps_timer(current_mode);
            previous_mode = current_mode;
        }

        switch (current_mode) {
            case TEST_MODE:
            case NORMAL_MODE:
                read_gps_data(&gps_data, measure, ctx);
                k_sem_give(ctx->main_gps_sem);
                wait_for_gps_event();
                break;

            default:  /* ADVANCED_MODE or others */
                k_timer_stop(&gps_timer);
                k_sem_give(ctx->main_gps_sem);
                k_sem_take(ctx->gps_sem, K_FOREVER);
                break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Thread Startup
 * ---------------------------------------------------------------------------*/

/**
 * @brief Start the GPS measurement thread.
 *
 * Initializes the GPS timer, semaphores, and creates the GPS thread
 * that continuously manages GPS data acquisition and synchronization
 * with the main thread.
 *
 * @param ctx Pointer to the shared @ref system_context structure.
 * @param measure Pointer to the shared @ref system_measurement structure.
 */
void start_gps_thread(struct system_context *ctx, struct system_measurement *measure) {
    k_sem_init(&gps_timer_sem, 0, 1);
    k_timer_init(&gps_timer, gps_timer_handler, NULL);

    k_thread_create(&gps_thread_data,
                    gps_stack,
                    K_THREAD_STACK_SIZEOF(gps_stack),
                    gps_thread_fn,
                    ctx, measure, NULL,
                    GPS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&gps_thread_data, "gps_thread");
}
