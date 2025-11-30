/**
 * @file main.c
 * @brief Plant Monitoring System main module.
 *
 * This module monitors plant conditions such as light, soil moisture,
 * temperature/humidity, acceleration, color, and GPS location.
 * It provides visual feedback through an RGB LED and allows the user
 * to switch operating modes via a button.
 *
 * ## Operating Modes:
 * - **TEST_MODE:** RGB LED shows the dominant color detected.
 * - **NORMAL_MODE:** Periodic measurements; RGB LED alerts if any sensor
 *   is out of range.
 * - **ADVANCED_MODE:** Periodic measurements; RGB LED shows real color.
 *
 * ## Button Behavior:
 * - Toggles between TEST, NORMAL, and ADVANCED modes with each press.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>

#include "main.h"
#include "sensors_thread.h"
#include "gps_thread.h"

/* --- Main Configuration -------------------------------------------------------- */
#define INITIAL_MODE TEST_MODE  /**< Initial operating mode at startup. */

#define TEST_PERIOD 2000      /**< Test mode measurement period in milliseconds. */
#define NORMAL_PERIOD 10000    /**< Normal mode measurement period in milliseconds. */

#define RGB_TIMER_PERIOD 500 /**< RGB LED timer period in milliseconds. */
#define STATS_TIMER_PERIOD 3600000 /**< Statistics reporting period (ms). */

#define PWM_STEP    1              /**< PWM step in milliseconds. */
#define PWM_PERIOD  15             /**< PWM period in milliseconds. */
#define PWM_STEPS   (PWM_PERIOD / PWM_STEP) /**< Number of PWM steps per period. */

/* --- Sensors Configuration -------------------------------------------------------- */
#define ACCEL_RANGE ACCEL_2G    /**< Accelerometer full-scale range setting. */

#define COLOR_GAIN GAIN_4X      /**< Color sensor gain setting. */
#define COLOR_INTEGRATION_TIME INTEGRATION_154MS /**< Color sensor integration time in milliseconds. */ 

#define TEMP_HUM_RESOLUTION TH_RES_RH12_TEMP14  /**< Temperature and humidity sensor resolution setting. */

/* --- Measurement Limits --------------------------------------------------- */
#define TEMP_MIN   -10   /**< Minimum temperature in °C. */
#define TEMP_MAX   50    /**< Maximum temperature in °C. */

#define HUM_MIN    25    /**< Minimum humidity percentage. */
#define HUM_MAX    75    /**< Maximum humidity percentage. */

#define LIGHT_MIN  0     /**< Minimum brightness percentage. */
#define LIGHT_MAX  100   /**< Maximum brightness percentage. */

#define MOISTURE_MIN   0    /**< Minimum soil moisture percentage. */
#define MOISTURE_MAX   100  /**< Maximum soil moisture percentage. */

#define COLOR_MIN     0    /**< Minimum raw color channel value. */
#define COLOR_MAX     65535 /**< Maximum raw color channel value. */

#define ACCEL_MIN  -2    /**< Minimum acceleration in g. */
#define ACCEL_MAX   2    /**< Maximum acceleration in g. */

/* --- Flags -------------------------------------------------------------------- */
#define FLAG_TEMP     (1U << 0)
#define FLAG_HUM      (1U << 1)
#define FLAG_LIGHT    (1U << 2)
#define FLAG_MOISTURE (1U << 3)
#define FLAG_COLOR    (1U << 4)
#define FLAG_ACCEL    (1U << 5)

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
 * @brief Soil moisture ADC configuration.
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
 * @brief Temperature and humidity sensor I2C configuration.
 */
static struct i2c_dt_spec th = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = TH_I2C_ADDR,
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
 * @brief Indicator LED bus configuration.
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

/* --- Semaphores ----------------------------------------------------------- */
static K_SEM_DEFINE(main_sem, 0, 1);
static K_SEM_DEFINE(main_sensors_sem, 0, 1);
static K_SEM_DEFINE(main_gps_sem, 0, 1);
static K_SEM_DEFINE(sensors_sem, 0, 1);
static K_SEM_DEFINE(gps_sem, 0, 1);

/* --- Data ----------------------------------------------------------------- */
/**
 * @enum system_mode_t
 * @brief System operating modes.
 *
 * These modes define how the system behaves:
 * - **TEST_MODE:** Shows the dominant detected color through the RGB LED.
 * - **NORMAL_MODE:** Periodically measures sensors and alerts if any reading
 *   is out of range.
 * - **ADVANCED_MODE:** Emulates PWM for RGB LED control.
 */
typedef enum {
    TEST_MODE = 0,    /**< Test mode – displays the dominant color. */
    NORMAL_MODE,      /**< Normal mode – periodic measurement and alerts. */
    ADVANCED_MODE     /**< Advanced mode – minimal visual feedback. */
} system_mode_t;

/**
 * @enum dom_color_t
 * @brief Dominant color types. 
 */
typedef enum {
    DOM_RED,
    DOM_GREEN,
    DOM_BLUE
} dom_color_t;

const char* dom_color_names[] = { "RED", "GREEN", "BLUE" };

/**
 * @brief Shared system context.
 *
 * The @ref system_context structure holds references to the peripheral
 * configurations. The main, sensor, and GPS threads use this structure
 * to coordinate configuration and mode updates.
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
};

/**
 * @brief Shared measurements between threads.
 *
 * The @ref system_measurement structure holds the current sensor readings
 * accessible to all threads.
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

/**
 * @brief Main data measurement structure.
 */
struct main_measurement {
    system_mode_t mode;
    float light;
    float moisture;
    float lat;
    float lon;
    float alt;
    float x_axis;
    float y_axis;
    float z_axis;
    float hum;
    float temp;
    int sats;
    int time_int;
    int hh, mm, ss;
    float c, r, g, b;
    char ns;
    char ew;
    dom_color_t dom_color;
    atomic_t rgb_flags;
};

/**
 * @brief Main measurement data initialization.
 */
static struct main_measurement main_data = {
    .mode = INITIAL_MODE,
    .light = 0.0f,
    .moisture = 0.0f,
    .lat = 0.0f,
    .lon = 0.0f,
    .alt = 0.0f,
    .x_axis = 0.0f,
    .y_axis = 0.0f,
    .z_axis = 0.0f,
    .hum = 0.0f,
    .temp = 0.0f,
    .sats = 0,
    .time_int = 0,
    .hh = 0,
    .mm = 0,
    .ss = 0,
    .c = 0,
    .r = 0,
    .b = 0,
    .g = 0,
    .ns = '\0',
    .ew = '\0',
    .dom_color = DOM_RED,
    .rgb_flags = ATOMIC_INIT(0),
};

/**
 * @brief Statistics data structure for mean, max, and min calculations.
 */
struct stats_measurements {
    float temp_mean, temp_max, temp_min;
    float hum_mean, hum_max, hum_min;
    float light_mean, light_max, light_min;
    float moisture_mean, moisture_max, moisture_min;
    float x_axis_max, x_axis_min;
    float y_axis_max, y_axis_min;
    float z_axis_max, z_axis_min;
    int red_count, green_count, blue_count;
    int count;
};

/**
 * @brief Statistics data initialization.
 */
struct stats_measurements stats_data = {0};

/* --- Button timing and state -------------------------------------------------- */
static struct k_work button_work;

/**
 * @brief Work handler for processing button press events.
 *
 * Determines the current operating mode and switches to the next one.
 *
 * @param work Pointer to the work structure.
 */
static void button_work_handler(struct k_work *work)
{
    switch (main_data.mode) {
        case TEST_MODE:
            main_data.mode = NORMAL_MODE;
            printk("\nNORMAL MODE\n");
            break;
        case NORMAL_MODE:
            main_data.mode = ADVANCED_MODE;
            printk("\nADVANCED MODE\n");
            break;
        case ADVANCED_MODE:
            main_data.mode = TEST_MODE;
            printk("\nTEST MODE\n");
            break;
    }

    k_sem_give(&main_sem);
}

/**
 * @brief ISR callback for button press/release events.
 *
 * Detects button release (active low) and submits work to handle mode toggling.
 *
 * @param dev Pointer to the GPIO device structure.
 * @param cb Pointer to the GPIO callback structure.
 * @param pins Bitmask of triggered pins.
 */
static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (!gpio_pin_get_dt(&button.spec)) {
        k_work_submit(&button_work);
    }
}

/* --- Main Timer -------------------------------------------------------- */
static struct k_timer main_timer;

/**
 * @brief Main periodic handler.
 *
 * Controls the time of each mode.
 *
 * @param timer Pointer to the main timer structure.
 */
static void main_timer_handler(struct k_timer *timer)
{
    k_sem_give(&main_sem);
}

/* --- RGB LED Timer -------------------------------------------------------- */
static struct k_timer rgb_timer;

/**
 * @brief RGB LED periodic handler for NORMAL_MODE.
 *
 * Cycles through RGB colors according to active out-of-range sensor flags.
 *
 * @param timer Pointer to the RGB timer structure.
 */
static void rgb_timer_handler(struct k_timer *timer)
{
    static uint8_t color_index = 0;
    uint32_t flags = atomic_get(&main_data.rgb_flags);

    uint8_t colors[6]; 
    uint8_t count = 0;

    if (flags & FLAG_TEMP)      colors[count++] = 0; // RED
    if (flags & FLAG_HUM)       colors[count++] = 1; // BLUE
    if (flags & FLAG_LIGHT)     colors[count++] = 2; // GREEN
    if (flags & FLAG_MOISTURE)  colors[count++] = 3; // CYAN
    if (flags & FLAG_COLOR)     colors[count++] = 4; // WHITE
    if (flags & FLAG_ACCEL)     colors[count++] = 5; // YELLOW

    if (count == 0) {
        rgb_led_off(&rgb_leds);
        color_index = 0;
        return;
    }

    uint8_t color = colors[color_index % count];
    color_index++;

    switch (color) {
        case 0: rgb_red(&rgb_leds); break;
        case 1: rgb_blue(&rgb_leds); break;
        case 2: rgb_green(&rgb_leds); break;
        case 3: rgb_cyan(&rgb_leds); break;
        case 4: rgb_white(&rgb_leds); break;
        case 5: rgb_yellow(&rgb_leds); break;
        default: rgb_led_off(&rgb_leds); break;
    }
}


/* --- Stats Timer -------------------------------------------------------- */
static struct k_timer stats_timer;

/**
 * @brief Stats periodic handler for NORMAL_MODE
 */
static void stats_timer_handler(struct k_timer *timer)
{
    if(main_data.mode == NORMAL_MODE) {
        printk("--- STATS REPORT ---\n");
        printk("Temperature: Mean: %.2f C, Max: %.2f C, Min: %.2f C\n",
                (double)stats_data.temp_mean, (double)stats_data.temp_max, (double)stats_data.temp_min);

        printk("Humidity: Mean: %.2f %%, Max: %.2f %%, Min: %.2f %%\n",
                (double)stats_data.hum_mean, (double)stats_data.hum_max, (double)stats_data.hum_min);

        printk("Light: Mean: %.2f %%, Max: %.2f %%, Min: %.2f %%\n",
                (double)stats_data.light_mean, (double)stats_data.light_max, (double)stats_data.light_min);

        printk("Soil Moisture: Mean: %.2f %%, Max: %.2f %%, Min: %.2f %%\n",
                (double)stats_data.moisture_mean, (double)stats_data.moisture_max, (double)stats_data.moisture_min);

        printk("Acceleration X-axis: Max: %.2f m/s2, Min: %.2f m/s2\n",
                (double)stats_data.x_axis_max * 9.8, (double)stats_data.x_axis_min * 9.8);

        printk("Acceleration Y-axis: Max: %.2f m/s2, Min: %.2f m/s2\n",
                (double)stats_data.y_axis_max * 9.8, (double)stats_data.y_axis_min * 9.8);

        printk("Acceleration Z-axis: Max: %.2f m/s2, Min: %.2f m/s2\n",
                (double)stats_data.z_axis_max * 9.8, (double)stats_data.z_axis_min * 9.8);

        if(stats_data.red_count >= stats_data.green_count && stats_data.red_count >= stats_data.blue_count) {
            printk("Dominant Color Detected: RED (%d times)\n", stats_data.red_count);
        } else if(stats_data.green_count >= stats_data.red_count && stats_data.green_count >= stats_data.blue_count) {
            printk("Dominant Color Detected: GREEN (%d times)\n", stats_data.green_count);
        } else {
            printk("Dominant Color Detected: BLUE (%d times)\n", stats_data.blue_count);
        }
        
        printk("---------------------\n\n");

        stats_data.temp_mean = 0.0f;
        stats_data.temp_max = 0.0f;
        stats_data.temp_min = 0.0f;

        stats_data.hum_mean = 0.0f;
        stats_data.hum_max = 0.0f;
        stats_data.hum_min = 0.0f;

        stats_data.light_mean = 0.0f;
        stats_data.light_max = 0.0f;
        stats_data.light_min = 0.0f;

        stats_data.moisture_mean = 0.0f;
        stats_data.moisture_max = 0.0f;
        stats_data.moisture_min = 0.0f;

        stats_data.x_axis_max = 0.0f;
        stats_data.x_axis_min = 0.0f;
        stats_data.y_axis_max = 0.0f;
        stats_data.y_axis_min = 0.0f;
        stats_data.z_axis_max = 0.0f;
        stats_data.z_axis_min = 0.0f;

        stats_data.red_count = 0;
        stats_data.green_count = 0;
        stats_data.blue_count = 0;

        stats_data.count = 0;
    }
}


/* --- Helper Functions ----------------------------------------------------- */

/**
 * @brief Calculates mean values for temperature, humidity, light and moisture.
 *
 * Updates the running mean based on the number of samples collected.
 */
static void mean_calculation()
{
    if (stats_data.count == 1) {
        stats_data.temp_mean = main_data.temp;
        stats_data.hum_mean = main_data.hum;
        stats_data.light_mean = main_data.light;
        stats_data.moisture_mean = main_data.moisture;
    } else {
        stats_data.temp_mean = ((stats_data.temp_mean * (stats_data.count - 1)) + main_data.temp) / stats_data.count;
        stats_data.hum_mean = ((stats_data.hum_mean * (stats_data.count - 1)) + main_data.hum) / stats_data.count;
        stats_data.light_mean = ((stats_data.light_mean * (stats_data.count - 1)) + main_data.light) / stats_data.count;
        stats_data.moisture_mean = ((stats_data.moisture_mean * (stats_data.count - 1)) + main_data.moisture) / stats_data.count;
    }
}

/**
 * @brief Calculates max/min values for all sensor measurements.
 *
 * Keeps track of the highest and lowest values recorded since last reset.
 */
static void max_min_calculation()
{
    if (stats_data.count == 1) {
        // Initialize first values
        stats_data.temp_max = stats_data.temp_min = main_data.temp;
        stats_data.hum_max = stats_data.hum_min = main_data.hum;
        stats_data.light_max = stats_data.light_min = main_data.light;
        stats_data.moisture_max = stats_data.moisture_min = main_data.moisture;

        stats_data.x_axis_max = stats_data.x_axis_min = main_data.x_axis;
        stats_data.y_axis_max = stats_data.y_axis_min = main_data.y_axis;
        stats_data.z_axis_max = stats_data.z_axis_min = main_data.z_axis;
    } else {
        // Temperature
        if (main_data.temp > stats_data.temp_max) stats_data.temp_max = main_data.temp;
        if (main_data.temp < stats_data.temp_min) stats_data.temp_min = main_data.temp;

        // Humidity
        if (main_data.hum > stats_data.hum_max) stats_data.hum_max = main_data.hum;
        if (main_data.hum < stats_data.hum_min) stats_data.hum_min = main_data.hum;

        // Light
        if (main_data.light > stats_data.light_max) stats_data.light_max = main_data.light;
        if (main_data.light < stats_data.light_min) stats_data.light_min = main_data.light;

        // Moisture
        if (main_data.moisture > stats_data.moisture_max) stats_data.moisture_max = main_data.moisture;
        if (main_data.moisture < stats_data.moisture_min) stats_data.moisture_min = main_data.moisture;

        // Accelerometer
        if (main_data.x_axis > stats_data.x_axis_max) stats_data.x_axis_max = main_data.x_axis;
        if (main_data.x_axis < stats_data.x_axis_min) stats_data.x_axis_min = main_data.x_axis;

        if (main_data.y_axis > stats_data.y_axis_max) stats_data.y_axis_max = main_data.y_axis;
        if (main_data.y_axis < stats_data.y_axis_min) stats_data.y_axis_min = main_data.y_axis;

        if (main_data.z_axis > stats_data.z_axis_max) stats_data.z_axis_max = main_data.z_axis;
        if (main_data.z_axis < stats_data.z_axis_min) stats_data.z_axis_min = main_data.z_axis;
    }
}

/**
 * @brief Updates the color counters based on the dominant color detected.
 *
 * Increments the count for the dominant color (red, green, or blue).
 */
static void dominant_color_calculation()
{
    if (main_data.r > main_data.g && main_data.r > main_data.b) {
        stats_data.red_count++;
    } else if (main_data.g > main_data.r && main_data.g > main_data.b) {
        stats_data.green_count++;
    } else if (main_data.b > main_data.r && main_data.b > main_data.g) {
        stats_data.blue_count++;
    }
}

/**
 * @brief Main statistics management handler.
 *
 * Increments sample count and updates mean, max/min, and color stats.
 */
static void stats_management()
{
    stats_data.count++;

    mean_calculation();

    max_min_calculation();

    dominant_color_calculation();
}

/**
 * @brief Checks if a value is within min/max limits.
 *
 * If the value is below min, sets it to min.
 * If the value is above max, sets it to max.
 * If the value was out-of-range, activates the corresponding flag.
 *
 * @param val Pointer to the value to check
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @param flags Pointer to flags variable
 * @param flag_bit Bit to set if value is out-of-range
 */
static void check_limit(float *val, float min, float max, uint32_t *flags, uint32_t flag_bit)
{
    if (*val < min) {
        *val = min;
        *flags |= flag_bit;
    } else if (*val > max) {
        *val = max;
        *flags |= flag_bit;
    }
}

/**
 * @brief Checks all the values are within min/max limits.
 */
static void check_limits(uint32_t *flags)
{
    *flags = 0U;

    check_limit(&main_data.temp, TEMP_MIN, TEMP_MAX, flags, FLAG_TEMP);
    check_limit(&main_data.hum, HUM_MIN, HUM_MAX, flags, FLAG_HUM);
    check_limit(&main_data.light, LIGHT_MIN, LIGHT_MAX, flags, FLAG_LIGHT);
    check_limit(&main_data.moisture, MOISTURE_MIN, MOISTURE_MAX, flags, FLAG_MOISTURE);

    check_limit(&main_data.c, COLOR_MIN, COLOR_MAX, flags, FLAG_COLOR);
    check_limit(&main_data.r, COLOR_MIN, COLOR_MAX, flags, FLAG_COLOR);
    check_limit(&main_data.g, COLOR_MIN, COLOR_MAX, flags, FLAG_COLOR);
    check_limit(&main_data.b, COLOR_MIN, COLOR_MAX, flags, FLAG_COLOR);

    check_limit(&main_data.x_axis, ACCEL_MIN * 9.8f, ACCEL_MAX * 9.8f, flags, FLAG_ACCEL);
    check_limit(&main_data.y_axis, ACCEL_MIN * 9.8f, ACCEL_MAX * 9.8f, flags, FLAG_ACCEL);
    check_limit(&main_data.z_axis, ACCEL_MIN * 9.8f, ACCEL_MAX * 9.8f, flags, FLAG_ACCEL);

    atomic_set(&main_data.rgb_flags, (atomic_val_t)(*flags));
}

/**
 * @brief Retrieves the latest measurements from atomic variables.
 */
static void get_measurements()
{
    main_data.moisture = atomic_get(&measure.moisture) / 10.0f;

    main_data.light = atomic_get(&measure.brightness) / 10.0f;

    main_data.lat  = atomic_get(&measure.gps_lat) / 1e6f;
    main_data.lon  = atomic_get(&measure.gps_lon) / 1e6f;
    main_data.alt  = atomic_get(&measure.gps_alt) / 100.0f;
    main_data.sats   = atomic_get(&measure.gps_sats);
    main_data.time_int = atomic_get(&measure.gps_time);

    main_data.ns = (main_data.lat >= 0) ? 'N' : 'S';
    main_data.ew = (main_data.lon >= 0) ? 'E' : 'W';

    main_data.lat = fabsf(main_data.lat);
    main_data.lon = fabsf(main_data.lon);

    if (main_data.time_int >= 0) {
        main_data.hh = main_data.time_int / 10000;
        main_data.mm = (main_data.time_int / 100) % 100;
        main_data.ss = main_data.time_int % 100;
    } else {
        printk("GPS time: --:--:--\n");
    }

    main_data.r = atomic_get(&measure.red);
    main_data.g = atomic_get(&measure.green);
    main_data.b = atomic_get(&measure.blue);
    main_data.c = atomic_get(&measure.clear);

    main_data.x_axis = atomic_get(&measure.accel_x_g) / 100.0f;
    main_data.y_axis = atomic_get(&measure.accel_y_g) / 100.0f;
    main_data.z_axis = atomic_get(&measure.accel_z_g) / 100.0f;

    main_data.temp = atomic_get(&measure.temp) / 100.0f;
    main_data.hum = atomic_get(&measure.hum) / 100.0f;
}

/**
 * @brief Displays the latest measurements via printk.
 */
static void display_measurements()
{
    printk("SOIL MOISTURE: %.1f%%\n", (double)main_data.moisture);

    printk("LIGHT: %.1f%%\n", (double)main_data.light);

    printk("GPS: #Sats: %d Lat(UTC): %.6f %c Long(UTC): %.6f %c Altitude: %.0f m GPS time: %02d:%02d:%02d\n",
            main_data.sats, (double)main_data.lat, main_data.ns, (double)main_data.lon, 
            main_data.ew, (double)main_data.alt, main_data.hh, main_data.mm, main_data.ss);

    printk("COLOR SENSOR: Clear: %.0f Red: %.0f Green: %.0f Blue: %.0f Dominant color: %s \n",
            (double)main_data.c, (double)main_data.r, (double)main_data.g, (double)main_data.b, dom_color_names[main_data.dom_color]);
                
    printk("ACCELEROMETER: X_axis: %.2f m/s2, Y_axis: %.2f m/s2, Z_axis: %.2f m/s2 \n",
            (double)main_data.x_axis, (double)main_data.y_axis, (double)main_data.z_axis);
                
    printk("TEMP/HUM: Temperature: %.1fC, Relative Humidity: %.1f%%\n\n",
            (double)main_data.temp, (double)main_data.hum);
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
    printk("System ON (TEST MODE)\n\n");

    uint32_t flags = 0;
    system_mode_t previous_mode = INITIAL_MODE;
    float r_norm = 0.0f, g_norm = 0.0f, b_norm = 0.0f;
    int r_duty = 0, g_duty = 0, b_duty = 0, r_value = 0, g_value = 0, b_value = 0;
    bool keep_running = true;

    /* Initialize peripherals */
    if (gps_init(&gps)) {
        printk("GPS initialization failed - Program stopped\n");
        return -1;
    }
    if (adc_init(&pt)) {
        printk("Phototransistor initialization failed - Program stopped\n");
        return -1;
    }
    if (adc_init(&sm)) {
        printk("Soil moisture sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (accel_init(&accel, ACCEL_RANGE)) {
        printk("Accelerometer initialization failed - Program stopped\n");
        return -1;
    }
    if (temp_hum_init(&th, TEMP_HUM_RESOLUTION)) {
        printk("Temperature/Humidity sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (color_init(&color, COLOR_GAIN, COLOR_INTEGRATION_TIME)) {
        printk("Color sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (led_init(&leds) || led_off(&leds)) {
        printk("LED initialization failed - Program stopped\n");
        return -1;
    }
    if (rgb_led_init(&rgb_leds) || rgb_led_off(&rgb_leds)) {
        printk("RGB LED initialization failed - Program stopped\n");
        return -1;
    }
    if (button_init(&button))  {
        printk("Button initialization failed - Program stopped\n");
        return -1;
    }
    if (button_set_callback(&button, button_isr)) {
        printk("Button callback setup failed - Program stopped\n");
        return -1;
    }

    /* Initialize timers */
    k_timer_init(&main_timer, main_timer_handler, NULL);
    k_timer_init(&rgb_timer, rgb_timer_handler, NULL);
    k_timer_init(&stats_timer, stats_timer_handler, NULL);

    k_timer_start(&main_timer, K_MSEC(TEST_PERIOD), K_MSEC(TEST_PERIOD));
    k_timer_start(&stats_timer, K_MSEC(STATS_TIMER_PERIOD), K_MSEC(STATS_TIMER_PERIOD));

    /* Button handling */
    k_work_init(&button_work, button_work_handler);

    /* Start measurement threads */
    start_sensors_thread(&ctx, &measure);
    start_gps_thread(&ctx, &measure);

    blue(&leds);

    while (1) {
        switch (main_data.mode) {

            case TEST_MODE:
                blue(&leds);

                if(previous_mode != TEST_MODE) {
                    k_timer_stop(&rgb_timer);
                    rgb_led_off(&rgb_leds);
                    k_timer_stop(&main_timer);
                    k_timer_start(&main_timer, K_MSEC(TEST_PERIOD), K_MSEC(TEST_PERIOD));
                    previous_mode = TEST_MODE;
                }

                k_sem_give(ctx.sensors_sem);
                k_sem_give(ctx.gps_sem);

                k_sem_take(ctx.main_sensors_sem, K_FOREVER);
                k_sem_take(ctx.main_gps_sem, K_FOREVER);

                get_measurements();

                if (main_data.r > main_data.g && main_data.r > main_data.b) {
                    rgb_red(&rgb_leds);
                    main_data.dom_color = DOM_RED;
                } else if (main_data.g > main_data.r && main_data.g > main_data.b) {
                    rgb_green(&rgb_leds);
                    main_data.dom_color = DOM_GREEN;
                } else {
                    rgb_blue(&rgb_leds);
                    main_data.dom_color = DOM_BLUE;
                }

                display_measurements();

                k_sem_take(&main_sem, K_FOREVER);

                break;

            case NORMAL_MODE:
                green(&leds);
                flags = 0;

                if(previous_mode != NORMAL_MODE) {
                    k_timer_stop(&main_timer);
                    k_timer_start(&main_timer, K_MSEC(NORMAL_PERIOD), K_MSEC(NORMAL_PERIOD));
                    k_timer_start(&rgb_timer, K_MSEC(RGB_TIMER_PERIOD), K_MSEC(RGB_TIMER_PERIOD));
                    previous_mode = NORMAL_MODE;
                }

                k_sem_give(ctx.sensors_sem);
                k_sem_give(ctx.gps_sem);

                k_sem_take(ctx.main_sensors_sem, K_FOREVER);
                k_sem_take(ctx.main_gps_sem, K_FOREVER);

                get_measurements();

                check_limits(&flags);

                stats_management();
                
                display_measurements();

                k_sem_take(&main_sem, K_FOREVER);
                
                break;

            case ADVANCED_MODE:
                red(&leds);

                if(previous_mode != ADVANCED_MODE) {
                    k_timer_stop(&rgb_timer);
                    rgb_led_off(&rgb_leds);

                    k_timer_stop(&main_timer);
                    k_timer_start(&main_timer, K_MSEC(NORMAL_PERIOD), K_MSEC(NORMAL_PERIOD));
                    previous_mode = ADVANCED_MODE;
                }

                k_sem_give(ctx.sensors_sem);
                k_sem_give(ctx.gps_sem);

                k_sem_take(ctx.main_sensors_sem, K_FOREVER);
                k_sem_take(ctx.main_gps_sem, K_FOREVER);

                get_measurements();

                display_measurements();

                if (main_data.c <= 0.0f) {
                    printk("[WARN] - Color clear channel == 0\n");
                    r_norm = g_norm = b_norm = 0.0f;
                } else {
                    r_norm = (main_data.r / main_data.c) * 100.0f;
                    g_norm = (main_data.g / main_data.c) * 100.0f;
                    b_norm = (main_data.b / main_data.c) * 100.0f;
                }

                printk("NORMALIZED COLOR VALUES: R: %.2f%%, G: %.2f%%, B: %.2f%%\n\n",
                        (double)r_norm, (double)g_norm, (double)b_norm);

                r_duty = (int)(r_norm * PWM_STEPS / 100);
                g_duty = (int)(g_norm * PWM_STEPS / 100);
                b_duty = (int)(b_norm * PWM_STEPS / 100);

                keep_running = true;
                while (keep_running) {
                    for (int t = 0; t < PWM_PERIOD; t+= PWM_STEP) {
                        r_value = (t < r_duty) ? 1 : 0;
                        g_value = (t < g_duty) ? 1 : 0;
                        b_value = (t < b_duty) ? 1 : 0;

                        rgb_led_pwm_step(&rgb_leds, r_value, g_value, b_value);
                        k_sleep(K_MSEC(PWM_STEP));                      

                        if (k_sem_take(&main_sem, K_NO_WAIT) == 0) {
                            keep_running = false;
                            break;
                        }
                    }
                }
                
                break;
        }
    }
}
