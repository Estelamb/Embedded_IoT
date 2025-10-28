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

    while (1) {
        /* Perform measurement only if system is in NORMAL mode */
        if (ctx->mode == NORMAL_MODE) {
            int32_t mv = 0;
            if (adc_read_voltage(&mv) == 0) {
                float percent = ((float)mv / ctx->adc->vref_mv) * 100.0f;
                if (percent > 100.0f) {
                    percent = 100.0f;
                } else if (percent < 0.0f) {
                    percent = 0.0f;
                }

                /* Update shared context safely */
                k_mutex_lock(&ctx->lock, K_FOREVER);
                ctx->brightness = percent;
                k_mutex_unlock(&ctx->lock);

                int int_part = (int)percent;
                int frac_part = (int)((percent - int_part) * 10); // 1 decimal

                printk("[BRIGHTNESS THREAD] Brightness: %d.%d%% (%d mV)\n", int_part, frac_part, mv);

            } else {
                printk("[BRIGHTNESS THREAD] ADC read error\n");
            }

            /* Wait the defined interval before the next measurement */
            k_sleep(K_MSEC(BRIGHTNESS_MEASURE_INTERVAL_MS));
        } else {
            /* Idle mode: sleep briefly before rechecking */
            k_sleep(K_MSEC(200));
        }
    }
}

/**
 * @brief Starts the brightness measurement thread.
 */
void start_brightness_thread(struct system_context *ctx)
{
    k_mutex_init(&ctx->lock);
    ctx->mode = OFF_MODE;

    k_thread_create(&brightness_thread_data,
                    brightness_stack,
                    K_THREAD_STACK_SIZEOF(brightness_stack),
                    brightness_thread_fn,
                    ctx, NULL, NULL,
                    BRIGHTNESS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&brightness_thread_data, "brightness_thread");
}
