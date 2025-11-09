/**
 * @file sensors_thread.c
 * @brief Implementation of the sensors measurement thread.
 */

#include "sensors_thread.h"
#include "adc.h"
#include "accel.h"
#include "temp_hum.h"
#include "color.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSORS_THREAD_STACK_SIZE 1024
#define SENSORS_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(sensors_stack, SENSORS_THREAD_STACK_SIZE);
static struct k_thread sensors_thread_data;

/* Timer and semaphore for periodic measurements */
static struct k_timer sensors_timer;
static struct k_sem sensors_timer_sem;

/* Poll events for timer and manual trigger */
static struct k_poll_event sensors_poll_events[2];

/* --------------------------------------------------------
 * Helper functions
 * --------------------------------------------------------*/

/**
 * @brief Timer handler: wakes up the thread when it expires.
 */
static void sensors_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&sensors_timer_sem);
}

/**
 * @brief Initialize the poll events for sensors thread.
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
 * @brief Waits for either timer or external semaphore event.
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
 * @brief Configure sensor timer based on current mode.
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
 * @brief Read ADC-based sensor (brightness, moisture).
 */
static void read_adc_percentage(const struct adc_config *cfg, atomic_t *target,
                                const char *label, int32_t *mv)
{
    if (adc_read_voltage(cfg, mv) == 0) {
        // porcentaje con 1 decimal, multiplicamos por 10
        int32_t percent10 = ((*mv) * 1000) / cfg->vref_mv; // 1000 = 100 * 10
        atomic_set(target, percent10);
    } else {
        printk("[ADC]: %s read error\n", label);
    }
}

/**
 * @brief Read accelerometer data and update measurement structure.
 *        Stored as scaled integers (value ×100).
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
 * @brief Read temperature and humidity sensor.
 *        Stored as scaled integers (×100).
 */
static void read_temperature_humidity(const struct i2c_dt_spec *dev,
                                      atomic_t *temp, atomic_t *hum) {
    float temperature, humidity;

    // 🔹 Primero leer humedad (mide ambos internamente)
    if (temp_hum_read_humidity(dev, &humidity) == 0) {
        // 🔹 Luego leer temperatura desde la última medición de RH
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
 * @brief Read color sensor data and update measurement structure.
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

/* --------------------------------------------------------
 * Thread main function
 * --------------------------------------------------------*/

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

/**
 * @brief Starts the sensors measurement thread.
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
