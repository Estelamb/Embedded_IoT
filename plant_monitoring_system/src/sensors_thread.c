/**
 * @file sensors_thread.c
 * @brief Implementation of the sensors measurement thread.
 *
 * This module defines the Zephyr thread responsible for periodically
 * acquiring data from multiple environmental sensors:
 * - **ADC sensors:** ambient brightness and soil moisture
 * - **I2C sensors:** accelerometer, temperature/humidity, and color sensor
 *
 * The thread’s activity depends on the current system mode:
 * - In @ref TEST_MODE or @ref NORMAL_MODE, periodic sampling is active.
 * - In @ref ADVANCED_MODE, the thread remains idle until reactivated.
 */

#include "sensors_thread.h"
#include "sensors/adc/adc.h"
#include "sensors/i2c/accel.h"
#include "sensors/i2c/temp_hum.h"
#include "sensors/i2c/color.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* --- Thread configuration --------------------------------------------------- */
#define SENSORS_THREAD_STACK_SIZE 1024  /**< Stack size allocated for the sensors thread. */
#define SENSORS_THREAD_PRIORITY   5     /**< Thread priority (lower = higher priority). */

K_THREAD_STACK_DEFINE(sensors_stack, SENSORS_THREAD_STACK_SIZE); /**< Thread stack for sensors task. */
static struct k_thread sensors_thread_data;                      /**< Thread control block for sensors. */

/* --- Timer and synchronization ---------------------------------------------- */
static struct k_timer sensors_timer;     /**< Periodic timer for sensor measurements. */
static struct k_sem sensors_timer_sem;   /**< Semaphore released when the timer expires. */

/** @brief Poll events for timer expiration and manual trigger. */
static struct k_poll_event sensors_poll_events[2];

/* ---------------------------------------------------------------------------
 * Helper functions
 * ---------------------------------------------------------------------------*/

/**
 * @brief Timer callback handler.
 *
 * Triggered automatically when the periodic sensor timer expires.
 * Wakes up the sensors thread by giving @ref sensors_timer_sem.
 *
 * @param timer_id Pointer to the timer instance that triggered the event.
 */
static void sensors_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&sensors_timer_sem);
}

/**
 * @brief Initialize poll events used by the sensors thread.
 *
 * Two poll events are used:
 * - **Index 0:** Timer expiration semaphore (@ref sensors_timer_sem)
 * - **Index 1:** External trigger semaphore (@ref system_context.sensors_sem)
 *
 * @param ctx Pointer to the shared @ref system_context.
 */
static void init_sensors_poll_events(struct system_context *ctx) {
    sensors_poll_events[0].type  = K_POLL_TYPE_SEM_AVAILABLE;
    sensors_poll_events[0].mode  = K_POLL_MODE_NOTIFY_ONLY;
    sensors_poll_events[0].sem   = &sensors_timer_sem;
    sensors_poll_events[0].state = K_POLL_STATE_NOT_READY;

    sensors_poll_events[1].type  = K_POLL_TYPE_SEM_AVAILABLE;
    sensors_poll_events[1].mode  = K_POLL_MODE_NOTIFY_ONLY;
    sensors_poll_events[1].sem   = ctx->sensors_sem;
    sensors_poll_events[1].state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Wait for either a timer event or a manual trigger event.
 *
 * This function blocks the sensors thread until one of the poll events
 * (timer expiration or semaphore release) becomes available.
 */
static void wait_for_sensors_event(void) {
    k_poll(sensors_poll_events, 2, K_FOREVER);

    if (sensors_poll_events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&sensors_timer_sem, K_NO_WAIT);
    }
    if (sensors_poll_events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(sensors_poll_events[1].sem, K_NO_WAIT);
    }

    sensors_poll_events[0].state = K_POLL_STATE_NOT_READY;
    sensors_poll_events[1].state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Configure the sensors timer based on the current mode.
 *
 * - **TEST_MODE:** Short measurement cadence for quick updates.
 * - **NORMAL_MODE:** Standard interval for regular operation.
 * - **ADVANCED_MODE:** Timer remains disabled.
 *
 * @param mode Current system operating mode.
 */
static void update_sensors_timer(system_mode_t mode) {
    k_timer_stop(&sensors_timer);

    switch (mode) {
        case TEST_MODE:
            k_timer_start(&sensors_timer, K_NO_WAIT, K_MSEC(TEST_MODE_CADENCE));
            break;
        case NORMAL_MODE:
            k_timer_start(&sensors_timer, K_NO_WAIT, K_MSEC(NORMAL_MODE_CADENCE));
            break;
        default:
            /* ADVANCED or other modes: timer disabled */
            break;
    }
}

/**
 * @brief Read an ADC sensor and store its value as a scaled percentage.
 *
 * Converts a raw ADC reading to a percentage (×10 for one decimal precision).
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @param target Pointer to the atomic variable where the scaled value will be stored.
 * @param label Descriptive name of the sensor (for logging).
 * @param mv Pointer to store the measured voltage (in millivolts).
 */
static void read_adc_percentage(const struct adc_config *cfg, atomic_t *target,
                                const char *label, int32_t *mv)
{
    if (adc_read_voltage(cfg, mv) == 0) {
        int32_t percent10 = ((*mv) * 1000) / cfg->vref_mv; /**< Scaled percentage ×10. */
        atomic_set(target, percent10);
    } else {
        printk("[ADC]: %s read error\n", label);
    }
}

/**
 * @brief Read accelerometer data and update the measurement structure.
 *
 * Converts raw XYZ data into acceleration (m/s² ×100) and stores them atomically.
 *
 * @param dev Pointer to the accelerometer I2C device specification.
 * @param range Accelerometer full-scale range setting.
 * @param x_ms2 Pointer to atomic variable for X-axis acceleration.
 * @param y_ms2 Pointer to atomic variable for Y-axis acceleration.
 * @param z_ms2 Pointer to atomic variable for Z-axis acceleration.
 */
static void read_accelerometer(const struct i2c_dt_spec *dev, uint8_t range,
                               atomic_t *x_ms2, atomic_t *y_ms2, atomic_t *z_ms2) {
    int16_t x_raw, y_raw, z_raw;
    float x_val, y_val, z_val;

    if (accel_read_xyz(dev, &x_raw, &y_raw, &z_raw) == 0) {
        accel_convert_to_ms2(x_raw, range, &x_val);
        accel_convert_to_ms2(y_raw, range, &y_val);
        accel_convert_to_ms2(z_raw, range, &z_val);

        atomic_set(x_ms2, (int32_t)(x_val * 100));
        atomic_set(y_ms2, (int32_t)(y_val * 100));
        atomic_set(z_ms2, (int32_t)(z_val * 100));
    } else {
        printk("[ACCELEROMETER] - Error reading accelerometer\n");
    }
}

/**
 * @brief Read temperature and humidity data.
 *
 * Reads relative humidity first (which triggers a temperature measurement internally),
 * then retrieves the corresponding temperature. Both values are scaled ×100.
 *
 * @param dev Pointer to the temperature/humidity I2C device specification.
 * @param temp Pointer to atomic variable for temperature (°C ×100).
 * @param hum Pointer to atomic variable for relative humidity (%RH ×100).
 */
static void read_temperature_humidity(const struct i2c_dt_spec *dev,
                                      atomic_t *temp, atomic_t *hum) {
    float temperature, humidity;

    if (temp_hum_read_humidity(dev, &humidity) == 0) {
        uint8_t buf[2];
        int ret = i2c_write_read_dt(dev, (uint8_t[]){ SI7021_READ_TEMP_FROM_RH }, 1, buf, 2);
        if (ret == 0) {
            uint16_t raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
            temperature = ((175.72f * raw_temp) / 65536.0f) - 46.85f;
        } else {
            printk("[TEMP/HUM SENSOR] - Error reading temperature from RH (%d)\n", ret);
            return;
        }

        atomic_set(hum,  (int32_t)(humidity * 100));
        atomic_set(temp, (int32_t)(temperature * 100));

    } else {
        printk("[TEMP/HUM SENSOR] - Read error (humidity)\n");
    }
}

/**
 * @brief Read RGB color sensor and update measurement structure.
 *
 * Reads raw RGB and clear channel values from the color sensor and updates
 * the shared measurement structure atomically.
 *
 * @param dev Pointer to the color sensor I2C device specification.
 * @param measure Pointer to the shared @ref system_measurement structure.
 */
static void read_color_sensor(const struct i2c_dt_spec *dev, struct system_measurement *measure) {
    ColorSensorData color_data;

    if (color_read_rgb(dev, &color_data) == 0) {
        atomic_set(&measure->red,   color_data.red);
        atomic_set(&measure->green, color_data.green);
        atomic_set(&measure->blue,  color_data.blue);
        atomic_set(&measure->clear, color_data.clear);
    } else {
        printk("[COLOR SENSOR] - Read error\n");
    }
}

/* ---------------------------------------------------------------------------
 * Sensors thread
 * ---------------------------------------------------------------------------*/

/**
 * @brief Main function for the sensors measurement thread.
 *
 * This thread continuously monitors the system mode and performs ADC and I2C
 * sensor readings when appropriate. The results are stored in the shared
 * @ref system_measurement structure.
 *
 * @param arg1 Pointer to the shared @ref system_context structure.
 * @param arg2 Pointer to the shared @ref system_measurement structure.
 * @param arg3 Unused (set to NULL).
 */
static void sensors_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx = (struct system_context *)arg1;
    struct system_measurement *measure = (struct system_measurement *)arg2;

    system_mode_t previous_mode = atomic_get(&ctx->mode);
    system_mode_t current_mode  = previous_mode;

    int32_t mv = 0;

    init_sensors_poll_events(ctx);
    update_sensors_timer(current_mode);

    while (1) {
        current_mode = atomic_get(&ctx->mode);

        /* Handle mode transitions */
        if (current_mode != previous_mode) {
            update_sensors_timer(current_mode);
            previous_mode = current_mode;
        }

        switch (current_mode) {
            case TEST_MODE:
            case NORMAL_MODE:
                read_adc_percentage(ctx->phototransistor, &measure->brightness, "Brightness", &mv);
                read_adc_percentage(ctx->soil_moisture, &measure->moisture, "Moisture", &mv);
                read_accelerometer(ctx->accelerometer, ctx->accel_range,
                                   &measure->accel_x_g, &measure->accel_y_g, &measure->accel_z_g);
                read_temperature_humidity(ctx->temp_hum, &measure->temp, &measure->hum);
                read_color_sensor(ctx->color, measure);

                k_sem_give(ctx->main_sensors_sem);
                wait_for_sensors_event();
                break;

            default:
                k_timer_stop(&sensors_timer);
                k_sem_give(ctx->main_sensors_sem);
                k_sem_take(ctx->sensors_sem, K_FOREVER);
                break;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Thread startup
 * ---------------------------------------------------------------------------*/

/**
 * @brief Start the sensors measurement thread.
 *
 * Initializes synchronization primitives and creates the Zephyr thread
 * that handles periodic sensor data acquisition.
 *
 * @param ctx Pointer to the shared @ref system_context structure.
 * @param measure Pointer to the shared @ref system_measurement structure.
 */
void start_sensors_thread(struct system_context *ctx, struct system_measurement *measure) {
    k_sem_init(&sensors_timer_sem, 0, 1);
    k_timer_init(&sensors_timer, sensors_timer_handler, NULL);

    k_thread_create(&sensors_thread_data,
                    sensors_stack,
                    K_THREAD_STACK_SIZEOF(sensors_stack),
                    sensors_thread_fn,
                    ctx, measure, NULL,
                    SENSORS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&sensors_thread_data, "sensors_thread");
}
