/**
 * @file gps_thread.c
 * @brief Implementation of the GPS measurement thread (with scaled values).
 */

#include "gps_thread.h"
#include "gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define GPS_THREAD_STACK_SIZE 1024
#define GPS_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
static struct k_thread gps_thread_data;

/* Timer and semaphore for periodic GPS measurements */
static struct k_timer gps_timer;
static struct k_sem gps_timer_sem;

/* Poll events for timer and manual trigger */
static struct k_poll_event gps_poll_events[2];

/* --------------------------------------------------------
 * Helper functions
 * --------------------------------------------------------*/

/**
 * @brief Timer handler: wakes up the thread when it expires.
 */
static void gps_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&gps_timer_sem);
}

/**
 * @brief Initialize the poll events for the GPS thread.
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
 * @brief Waits for whichever occurs first: gps_timer_sem or ctx->gps_sem.
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
 * @brief Configure GPS timer based on current mode.
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
 * @brief Reads GPS data and updates the measurement structure.
 * @param data Pointer to a persistent gps_data_t buffer.
 */
static void read_gps_data(gps_data_t *data,
                          struct system_measurement *measure,
                          struct system_context *ctx) {

    if (gps_wait_for_gga(data, K_MSEC(2000)) == 0) {
        atomic_set(&measure->gps_lat,  (int32_t)(data->lat  * 1e6f));
        atomic_set(&measure->gps_lon,  (int32_t)(data->lon  * 1e6f));
        atomic_set(&measure->gps_alt,  (int32_t)(data->alt  * 100.0f));
        atomic_set(&measure->gps_sats, (int32_t)data->sats);
        atomic_set(&measure->gps_hdop, (int32_t)(data->hdop * 100.0f));
    } else {
        printk("[GPS] - Timeout or invalid data\n");
    }
}

/* --------------------------------------------------------
 * Thread main function
 * --------------------------------------------------------*/

/**
 * @brief GPS measurement thread function.
 */
static void gps_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx     = (struct system_context *)arg1;
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

/* --------------------------------------------------------
 * Thread startup
 * --------------------------------------------------------*/

/**
 * @brief Starts the GPS measurement thread.
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
