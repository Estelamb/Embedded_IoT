/**
 * to agps and cogpsrols an RGB LED accordingly. A user button toggles
 * @file main.c
 * @brief Main application for the interrupt-driven brightness control system.
 *
 * This application reads ambient light using a phototransistor connected
 * the operating mode (OFF, NORMAL, BLUE). Button presses are processed
 * via interrupts and deferred work. Press duration determines action.
 *
 * A brightness thread autonomously performs periodic brightness
 * measurements in NORMAL mode.
 *
 * Button behavior:
 * - Short press (< 1 s): Toggles between NORMAL and BLUE modes.
 * - Long press (≥ 1 s): Turns the system ON or OFF immediately.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "main.h"
#include "sensors_thread.h"
#include "gps_thread.h"

#define INITIAL_MODE TEST_MODE /**< Initial operating mode at startup. */
#define ACCEL_RANGE ACCEL_2G    /**< Accelerometer full-scale range setting. */

/* --- Peripheral configuration ------------------------------------------------ */

/**
 * @brief Phototransistor ADC configuration.
 */
static struct adc_config pt = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
    .channel_id = 5,
    .resolution = 12,
    .gain = ADC_GAIN_1,
    .ref = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .vref_mv = 3300,
};

/**
 * @brief Soil Moisture ADC configuration.
 */
static struct adc_config sm = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
    .channel_id = 0,    
    .resolution = 12,
    .gain = ADC_GAIN_1,
    .ref = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .vref_mv = 3300,
};

/**
 * @brief Accelerometer I2C configuration.
 */
static struct i2c_dt_spec accel = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = ACCEL_I2C_ADDR,
};

/**
 * @brief Temperature and Humidity sensor I2C configuration.
 */
static struct i2c_dt_spec th = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = TEMP_HUM_I2C_ADDR,
};

/**
 * @brief Color sensor I2C configuration.
 */
static struct i2c_dt_spec color = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = COLOR_I2C_ADDR,
};

/**
 * @brief GPS UART configuration.
 */
static struct gps_config gps = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(usart1)),
};

/**
 * @brief RGB LED bus configuration.
 */
static struct bus_rgb_led rgb_leds = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(red), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(green), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(blue), gpios)
    },
    .pin_count = BUS_SIZE,
};

/**
 * @brief LED bus configuration.
 */
static struct bus_led leds = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios)
    },
    .pin_count = BUS_SIZE,
};

/**
 * @brief User button configuration.
 */
static struct user_button button = {
    .spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
};

/* Semaphore to trigger and read measurements in NORMAL mode */
static K_SEM_DEFINE(main_sensors_sem, 0, 1);
static K_SEM_DEFINE(main_gps_sem, 0, 1);
static K_SEM_DEFINE(sensors_sem, 0, 1);
static K_SEM_DEFINE(gps_sem, 0, 1);

/**
 * @brief Shared context with the brightness thread.
 *
 * The @ref system_context structure holds references to the peripheral configuration.
 * Both the main thread and the brightness thread use this structure to
 * coordinate mode and brightness updates.
 */
static struct system_context ctx = {
    .phototransistor = &pt,

    .soil_moisture = &sm,

    .accelerometer = &accel,
    .accel_range = ACCEL_RANGE,

    .temp_hum = &th,

    .color = &color,

    .gps = &gps,

    .main_sensors_sem = &main_sensors_sem,
    .main_gps_sem = &main_gps_sem,
    .sensors_sem = &sensors_sem,
    .gps_sem = &gps_sem,

    .mode = ATOMIC_INIT(INITIAL_MODE),
};

/**
 * @brief Shared measurements with the brightness thread.
 *
 * The @ref system_measurement structure holds references to the peripheral configuration.
 * Both the main thread and the brightness thread use this structure to
 * coordinate mode and brightness updates.
 */
static struct system_measurement measure = {
    .brightness = ATOMIC_INIT(0),

    .moisture = ATOMIC_INIT(0),

    .accel_x_g = ATOMIC_INIT(0),
    .accel_y_g = ATOMIC_INIT(0),
    .accel_z_g = ATOMIC_INIT(0),

    .temp = ATOMIC_INIT(0),
    .hum = ATOMIC_INIT(0),

    .red = ATOMIC_INIT(0),
    .green = ATOMIC_INIT(0),
    .blue = ATOMIC_INIT(0),
    .clear = ATOMIC_INIT(0),

    .gps_lat = ATOMIC_INIT(0),
    .gps_lon = ATOMIC_INIT(0),
    .gps_alt = ATOMIC_INIT(0),
    .gps_sats = ATOMIC_INIT(0),
    .gps_time = ATOMIC_INIT(0),
};


/* --- Button timing and state -------------------------------------------------- */
static struct k_work button_work;

/**
 * @brief Deferred work handler for processing button events.
 *
 * Determines if the event to process is a short or long press
 * based on the state of long_press_fired flag.
 */
static void button_work_handler(struct k_work *work)
{
    system_mode_t current_mode = atomic_get(&ctx.mode);
    system_mode_t next_mode;

    switch (current_mode) {
        case TEST_MODE:
            next_mode = NORMAL_MODE;
            printk("\nNORMAL MODE\n");
            break;
        case NORMAL_MODE:
            next_mode = ADVANCED_MODE;
            printk("\nADVANCED MODE\n");
            break;
        case ADVANCED_MODE:
        default:
            next_mode = TEST_MODE;
            printk("\nTEST MODE\n");
            break;
    }

    atomic_set(&ctx.mode, next_mode);
    k_sem_give(ctx.sensors_sem);
    k_sem_give(ctx.gps_sem);
}

/**
 * @brief ISR callback for button press/release events.
 *
 * Detects the press (active low) and release edges and manages
 * the press timer accordingly.
 */
static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* Detect release (button unpressed) */
    if (!gpio_pin_get_dt(&button.spec)) {
        k_work_submit(&button_work);
    }
}


/* --- Main Application -------------------------------------------------------- */

/**
 * @brief Main entry point for the brightness control system.
 *
 * Initializes peripherals (RGB LED, ADC, user button), starts the
 * brightness thread, and executes the LED update loop.
 *
 * Button input is interrupt-driven; all press logic is handled by ISR and workqueue.
 *
 * @return This function does not return under normal operation.
 */
int main(void)
{
    printk("==== Plant Monitoring System ====\n");
    system_mode_t mode = INITIAL_MODE;
    float light, moisture, lat, lon, alt, x_axis, y_axis, z_axis, hum, temp = 0;
    int sats, time_int, hh, mm, ss, c, r, b, g = 0;
    char dom_color[6], ns, ew;

    /* Initialize peripherals */
    if (gps_init(&gps)) return -1;
    if (adc_init(&pt)) return -1;
    if (adc_init(&sm)) return -1;
    if (accel_init(&accel, ACCEL_RANGE)) return -1;
    if (temp_hum_init(&th)) return -1;
    if (color_init(&color)) return -1;
    if (led_init(&leds) || led_off(&leds)) return -1;
    if (rgb_led_init(&rgb_leds) || rgb_led_off(&rgb_leds)) return -1;
    if (button_init(&button)) return -1;
    if (button_set_callback(&button, button_isr)) return -1;

    /* Button handling */
    k_work_init(&button_work, button_work_handler);

    /* Start measurement threads */
    start_sensors_thread(&ctx, &measure);
    start_gps_thread(&ctx, &measure);

    printk("System ON (TEST MODE)\n");

    while (1) {
        mode = atomic_get(&ctx.mode);

        if (mode != ADVANCED_MODE) {
            k_sem_take(ctx.main_sensors_sem, K_FOREVER);
            k_sem_take(ctx.main_gps_sem, K_FOREVER);
        }

        switch (mode) {
            case TEST_MODE:
            case NORMAL_MODE:
                moisture = atomic_get(&measure.moisture) / 10.0f;

                light = atomic_get(&measure.brightness) / 10.0f;

                lat  = atomic_get(&measure.gps_lat) / 1e6f;
                lon  = atomic_get(&measure.gps_lon) / 1e6f;
                alt  = atomic_get(&measure.gps_alt) / 100.0f;
                sats   = atomic_get(&measure.gps_sats);
                time_int = atomic_get(&measure.gps_time);

                ns = (lat >= 0) ? 'N' : 'S';
                ew = (lon >= 0) ? 'E' : 'W';

                lat = fabsf(lat);
                lon = fabsf(lon);

                if (time_int >= 0) {
                    hh = time_int / 10000;
                    mm = (time_int / 100) % 100;
                    ss = time_int % 100;
                } else {
                    printk("GPS time: --:--:--\n");
                }
                r = atomic_get(&measure.red);
                g = atomic_get(&measure.green);
                b = atomic_get(&measure.blue);
                c = atomic_get(&measure.clear);

                x_axis = atomic_get(&measure.accel_x_g) / 100.0f;
                y_axis = atomic_get(&measure.accel_y_g) / 100.0f;
                z_axis = atomic_get(&measure.accel_z_g) / 100.0f;

                temp = atomic_get(&measure.temp) / 100.0f;
                hum = atomic_get(&measure.hum) / 100.0f;

                if (mode == TEST_MODE) {
                    if (r > g && r > b) {
                        rgb_red(&rgb_leds);
                        strcpy(dom_color, "RED");
                    } else if (g > r && g > b) {
                        rgb_green(&rgb_leds);
                        strcpy(dom_color, "GREEN");
                    } else {
                        rgb_blue(&rgb_leds);
                        strcpy(dom_color, "BLUE");
                    }

                } else {
                    green(&leds);
                }
                
                printk("SOIL MOISTURE: %.1f%%\n", moisture);

                printk("LIGHT: %.1f%%\n", light);

                printk("GPS: #Sats: %d Lat(UTC): %.6f %c Long(UTC): %.6f %c Altitude: %.0f m GPS time: %02d:%02d:%02d\n",
                        sats, lat, ns, lon, ew, alt,  hh, mm, ss);
                
                printk("COLOR SENSOR: Clear: %d Red: %d Green: %d Blue: %d Dominant color: %s \n",
                        c, r, g, b, dom_color);
                
                printk("ACCELEROMETER: X_axis: %.2f m/s2, Y_axis: %.2f m/s2, Z_axis: %.2f m/s2 \n",
                        x_axis, y_axis, z_axis);
                
                printk("TEMP/HUM: Temperature: %.1f C, Relative Humidity: %.1f%%\n\n",
                        temp, hum);
                
                break;

            case ADVANCED_MODE:
                red(&leds);
                break;
        }

        k_sem_take(ctx.main_sensors_sem, K_FOREVER);
        k_sem_take(ctx.main_gps_sem, K_FOREVER);
    }
}
