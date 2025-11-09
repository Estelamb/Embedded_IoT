/**
 * @file sensors_thread.c
 * @brief Implementation of the sensors measurement thread.
 *
 * The sensors thread runs continuously, checking the current
 * operating mode stored in the shared context. When the mode is NORMAL,
 * it periodically reads ADC values corresponding to brightness and soil moisture,
 * converts them to percentages, and updates the shared context.
 */

#include "sensors_thread.h"
#include "adc.h"
#include "accel.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSORS_THREAD_STACK_SIZE 1024
#define SENSORS_THREAD_PRIORITY 5
#define SENSORS_MEASURE_INTERVAL_MS 2000 /**< Measurement interval in NORMAL mode. */

/** Stack allocation for the sensors thread. */
K_THREAD_STACK_DEFINE(sensors_stack, SENSORS_THREAD_STACK_SIZE);

/** Thread control block. */
static struct k_thread sensors_thread_data;

/* Timer to trigger periodic sensor measurements */
static struct k_timer sensors_timer;

/* Semaphore to wake up the thread when the timer expires */
static struct k_sem sensors_timer_sem;

/**
 * @brief Timer handler: Gives the semaphore when the timer expires.
 */
static void sensors_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&sensors_timer_sem);
}

/**
 * @brief Read ADC sensor and convert to percentage.
 *
 * This function reads a voltage value from an ADC-configured sensor,
 * converts the voltage to a percentage (0-100), and writes it to a
 * target atomic variable.
 *
 * @param cfg Pointer to the ADC configuration for the sensor.
 * @param target Pointer to the atomic variable to store the result.
 * @param label Label string for debugging output.
 */
static void read_adc_percentage(const struct adc_config *cfg, atomic_t *target, const char *label, int32_t *mv, uint8_t *percent) {
    if (adc_read_voltage(cfg, mv) == 0) {
        *percent = (*mv * 100) / cfg->vref_mv;
        if (*percent > 100) *percent = 100;

        atomic_set(target, *percent);

        printk("[SENSORS THREAD] %s: %d%% (%d mV)\n", label, *percent, *mv);
    }
}

static void read_accelerometer(const struct i2c_dt_spec *dev, uint8_t range,
                               atomic_t *x_ms2, atomic_t *y_ms2, atomic_t *z_ms2) {
    int16_t x_raw, y_raw, z_raw;
    float x_val, y_val, z_val;

    if (accel_read_xyz(dev, &x_raw, &y_raw, &z_raw) == 0) {
        accel_convert_to_ms2(x_raw, range, &x_val);
        accel_convert_to_ms2(y_raw, range, &y_val);
        accel_convert_to_ms2(z_raw, range, &z_val);

        printk("[ACCEL] X=%.3f m/s^2, Y=%.3f m/s^2, Z=%.3f m/s^2\n",
               x_val, y_val, z_val);

        atomic_set(x_ms2, *(atomic_t *)&x_val);
        atomic_set(y_ms2, *(atomic_t *)&y_val);
        atomic_set(z_ms2, *(atomic_t *)&z_val);
    } else {
        printk("[ACCEL] Error reading accelerometer\n");
    }
}

/**
 * @brief Sensors measurement thread function.
 *
 * Periodically checks the current operating mode. When the system is
 * in NORMAL mode, this thread measures sensor values, converts them
 * to percentages, and updates context variables atomically.
 *
 * @param arg1 Pointer to a @ref system_context structure.
 * @param arg2 Pointer to a @ref system_measurement structure.
 * @param arg3 Unused.
 */
static void sensors_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx = (struct system_context *)arg1;
    struct system_measurement *measure = (struct system_measurement *)arg2;
    system_mode_t previous_mode = atomic_get(&ctx->mode);
    system_mode_t actual_mode = previous_mode;

    /* Shared working variables */
    int32_t mv = 0;
    uint8_t percent = 0;

    if (actual_mode == NORMAL_MODE) {
        k_timer_start(&sensors_timer, K_NO_WAIT, K_MSEC(SENSORS_MEASURE_INTERVAL_MS));
    }

    while (1) {
        actual_mode = atomic_get(&ctx->mode);

        if (actual_mode == NORMAL_MODE) {
            if (previous_mode != NORMAL_MODE) {
                k_timer_start(&sensors_timer, K_NO_WAIT, K_MSEC(SENSORS_MEASURE_INTERVAL_MS));
            }

            previous_mode = NORMAL_MODE;

            /* Read brightness and moisture */
            mv = 0;
            read_adc_percentage(ctx->phototransistor, &measure->brightness, "Brightness", &mv, &percent);

            mv = 0;
            read_adc_percentage(ctx->soil_moisture, &measure->moisture, "Moisture", &mv, &percent);

            /* Read accelerometer */
            read_accelerometer(ctx->accelerometer, ctx->accel_range, &measure->accel_x_g, &measure->accel_y_g, &measure->accel_z_g);

            /* Read Temperature and Humidity */

            /* Read Color */

            /* Wait for next measurement */
            k_sem_take(&sensors_timer_sem, K_FOREVER);
        } else {
            if (previous_mode == NORMAL_MODE) {
                k_timer_stop(&sensors_timer);
            }

            previous_mode = actual_mode;
            k_sem_take(ctx->sensors_sem, K_FOREVER);
        }
    }
}

/**
 * @brief Starts the sensors measurement thread.
 *
 * This function initializes core components and starts the thread
 * which will run the sensor polling loop.
 *
 * @param ctx Pointer to a valid @ref system_context structure.
 * @param measure Pointer to a valid @ref system_measurement structure.
 */
void start_sensors_thread(struct system_context *ctx, struct system_measurement *measure) {
    /* Init timer and measurement semaphore */
    k_sem_init(&sensors_timer_sem, 0, 1);
    k_timer_init(&sensors_timer, sensors_timer_handler, NULL);

    /* Start thread */
    k_thread_create(&sensors_thread_data,
                    sensors_stack,
                    K_THREAD_STACK_SIZEOF(sensors_stack),
                    sensors_thread_fn,
                    ctx, measure, NULL,
                    SENSORS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&sensors_thread_data, "sensors_thread");
}
