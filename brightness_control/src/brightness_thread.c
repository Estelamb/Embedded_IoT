/**
 * @file brightness_thread.c
 * @brief Implementation of the brightness measurement thread.
 *
 * The brightness thread runs continuously, checking the current
 * operating mode stored in the shared context. When the mode is NORMAL,
 * it periodically reads the ADC value corresponding to the ambient
 * light level, converts it to a percentage, and updates the shared context.
 */

#include "brightness_thread.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define BRIGHTNESS_THREAD_STACK_SIZE 1024
#define BRIGHTNESS_THREAD_PRIORITY 5
#define BRIGHTNESS_MEASURE_INTERVAL_MS 2000 /**< Measurement interval when in NORMAL mode. */

/** Stack allocation for the brightness thread. */
K_THREAD_STACK_DEFINE(brightness_stack, BRIGHTNESS_THREAD_STACK_SIZE);

/** Thread control block. */
static struct k_thread brightness_thread_data;

/* Timer to trigger brightness measurements */
static struct k_timer brightness_timer;

/* Semaphore to wake up the thread when the timer expires */
static struct k_sem brightness_timer_sem;

/* Timer handler: Give semaphore when the timer expires */
static void brightness_timer_handler(struct k_timer *timer_id)
{
    k_sem_give(&brightness_timer_sem);
}

/**
 * @brief Brightness measurement thread function.
 *
 * Periodically checks the current operating mode. When the system is
 * in NORMAL mode, the thread performs an ADC measurement, computes
 * brightness in percentage, and updates the shared context.
 *
 * @param arg1 Pointer to a @ref system_context structure.
 * @param arg2 Unused.
 * @param arg3 Unused.
 */
static void brightness_thread_fn(void *arg1, void *arg2, void *arg3)
{
    struct system_context *ctx = (struct system_context *)arg1;
    int previous_mode = atomic_get(&ctx->mode);
    int actual_mode = atomic_get(&ctx->mode);

    if (actual_mode == NORMAL_MODE) {
        k_timer_start(&brightness_timer, K_NO_WAIT, K_MSEC(BRIGHTNESS_MEASURE_INTERVAL_MS));
    }

    while (1) {
        actual_mode = atomic_get(&ctx->mode);

        /* Perform measurement only if system is in NORMAL mode */
        if (actual_mode == NORMAL_MODE) {
            if (previous_mode != NORMAL_MODE) {
                k_timer_start(&brightness_timer, K_NO_WAIT, K_MSEC(BRIGHTNESS_MEASURE_INTERVAL_MS));
            }

            previous_mode = NORMAL_MODE;

            int32_t mv = 0;
            if (adc_read_voltage(&mv) == 0) {
                float percent = ((float)mv / ctx->phototransistor->vref_mv) * 100.0f;
                if (percent < 0) percent = 0;
                if (percent > 100) percent = 100;
            
                atomic_set(&ctx->brightness, (int)percent);
            
                int int_part = (int)percent;
                int frac_part = (int)((percent - int_part) * 10); // 1 decimal

                printk("[BRIGHTNESS THREAD] Brightness: %d.%d%% (%d mV)\n", int_part, frac_part, mv);
            }

            /* Wait the defined interval before the next measurement */
            k_sem_take(&brightness_timer_sem, K_FOREVER);
        } else {
            if (previous_mode == NORMAL_MODE) {
                k_timer_stop(&brightness_timer);
            }

            previous_mode = actual_mode;
            
            /* Wait until NORMAL mode is activated */
            k_sem_take(ctx->brightness_sem, K_FOREVER);
        }
    }
}

/**
 * @brief Starts the brightness measurement thread.
 */
void start_brightness_thread(struct system_context *ctx)
{
    /* Init timer and measurement semaphore */
    k_sem_init(&brightness_timer_sem, 0, 1);
    k_timer_init(&brightness_timer, brightness_timer_handler, NULL);

    /* Start thread */
    k_thread_create(&brightness_thread_data,
                    brightness_stack,
                    K_THREAD_STACK_SIZEOF(brightness_stack),
                    brightness_thread_fn,
                    ctx, NULL, NULL,
                    BRIGHTNESS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&brightness_thread_data, "brightness_thread");
}
