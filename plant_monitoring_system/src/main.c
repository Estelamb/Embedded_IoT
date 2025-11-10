/**
 * to agps and cogpsrols an RGB LED accordingly. A user button toggles
 * @file main.c
 * @brief Plant Monitoring System main module
 *
 * Monitors plant conditions (light, soil moisture, temperature/humidity,
 * accelerometer, color sensor) and GPS location. Provides RGB LED visual
 * feedback and button-controlled operating modes.
 *
 * Operating modes:
 * - TEST_MODE: RGB indicates dominant color.
 * - NORMAL_MODE: Periodic measurements, RGB alerts for out-of-range sensors.
 * - ADVANCED_MODE: Minimal feedback, system ON.
 *
 * Button behavior:
 * - Toogles between TEST, NORMAL, and ADVANCED modes on each press.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>

#include "main.h"
#include "sensors_thread.h"
#include "gps_thread.h"

/* --- Configuration -------------------------------------------------------- */
#define INITIAL_MODE TEST_MODE /**< Initial operating mode at startup. */
#define ACCEL_RANGE ACCEL_2G    /**< Accelerometer full-scale range setting. */
#define RGB_TIMER_PERIOD_MS 500  /**< RGB LED timer period in milliseconds. */

/* Measurement Limits */

#define TEMP_MIN   -10   /**< Min temp. -10.0 C */
#define TEMP_MAX   50    /**< Max temp.  50.0 C */

#define HUM_MIN    25    /**< Min humidity 25.0 % */
#define HUM_MAX    75    /**< Max humidity 75.0 % */

#define LIGHT_MIN  0     /**< Min brightness 0.0 %  */
#define LIGHT_MAX  100   /**< Max brightness 100.0 % */

#define MOISTURE_MIN   0    /**< Min moisture 0.0 % */
#define MOISTURE_MAX   100  /**< Max moisture 100.0 % */

#define COLOR_CLEAR_MIN   1    /**< Min color clear channel raw value (dark threshold) */
#define COLOR_CLEAR_MAX   5000  /**< Max color clear channel raw value (saturation threshold) */
#define RED_MIN     0    /**< Min red channel raw value */
#define RED_MAX     5000  /**< Max red channel raw value */
#define GREEN_MIN   0    /**< Min green channel raw value */
#define GREEN_MAX   5000  /**< Max green channel raw value */
#define BLUE_MIN    0    /**< Min blue channel raw value */
#define BLUE_MAX    5000  /**< Max blue channel raw value */

#define ACCEL_MIN  -2    /**< Min accel. -2.00 g */
#define ACCEL_MAX  2     /**< Max accel.  2.00 g */

/* Flags for out-of-range conditions */
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

/* --- Semaphores ----------------------------------------------------------- */
static K_SEM_DEFINE(main_sensors_sem, 0, 1);
static K_SEM_DEFINE(main_gps_sem, 0, 1);
static K_SEM_DEFINE(sensors_sem, 0, 1);
static K_SEM_DEFINE(gps_sem, 0, 1);

/**
 * @brief Shared system context.
 *
 * The @ref system_context structure holds references to the peripheral configuration.
 * Both the main, the sensors and gps threads use this structure to
 * coordinate peripheral configurations and mode updates.
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
 * @brief Shared measurements.
 *
 * The @ref system_measurement structure holds references to the measurements.
 * Both the main, the sensors and gps threads use this structure to
 * coordinate measurements updates.
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
 * @brief Main measurements.
 *
 * The @ref main_measurement structure holds references to the main measurements.
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
    char dom_color[6];
    atomic_t rgb_flags;
};

/**
 * @brief Main data measurements Initialization.
 *
 * The @ref main_data structure holds references to the main measurements.
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
    .dom_color = {0, 0, 0, 0, 0, 0},
    .rgb_flags = ATOMIC_INIT(0),
};

/* --- Button timing and state -------------------------------------------------- */
static struct k_work button_work;

/**
 * @brief Work handler for processing button events.
 * 
 * Detects the current mode and switches to the next one in sequence.
 *
 * @param work Pointer to work struct
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
    k_sem_give(ctx.main_sensors_sem);
    k_sem_give(ctx.main_gps_sem);
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

/* --- RGB LED Timer -------------------------------------------------------- */
static struct k_timer rgb_timer;
static bool rgb_timer_running = false;

/**
 * @brief RGB LED periodic handler for NORMAL_MODE
 */
static void rgb_timer_handler(struct k_timer *timer)
{
    static uint8_t color_index = 0;
    uint32_t flags = atomic_get(&main_data.rgb_flags);

    uint8_t colors[8], count = 0;

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

/* --- Helper Functions ----------------------------------------------------- */
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
static void check_min_max(float *val, float min, float max, uint32_t *flags, uint32_t flag_bit)
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
            (double)main_data.c, (double)main_data.r, (double)main_data.g, (double)main_data.b, main_data.dom_color);
                
    printk("ACCELEROMETER: X_axis: %.2f m/s2, Y_axis: %.2f m/s2, Z_axis: %.2f m/s2 \n",
            (double)main_data.x_axis, (double)main_data.y_axis, (double)main_data.z_axis);
                
    printk("TEMP/HUM: Temperature: %.1f C, Relative Humidity: %.1f%%\n\n",
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
    uint32_t flags = 0;

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

    /* Initialize timer RGB */
    k_timer_init(&rgb_timer, rgb_timer_handler, NULL);

    /* Button handling */
    k_work_init(&button_work, button_work_handler);

    /* Start measurement threads */
    start_sensors_thread(&ctx, &measure);
    start_gps_thread(&ctx, &measure);

    blue(&leds);
    printk("System ON (TEST MODE)\n\n");

    while (1) {
        main_data.mode = atomic_get(&ctx.mode);

        if (main_data.mode != ADVANCED_MODE) {
            k_sem_take(ctx.main_sensors_sem, K_FOREVER);
            k_sem_take(ctx.main_gps_sem, K_FOREVER);
        }

        switch (main_data.mode) {
            case TEST_MODE:
                blue(&leds);

                if (rgb_timer_running) {
                    rgb_timer_running = false;
                    k_timer_stop(&rgb_timer);
                    rgb_led_off(&rgb_leds);
                }

                get_measurements();

                if (main_data.r > main_data.g && main_data.r > main_data.b) {
                    rgb_red(&rgb_leds);
                    strcpy(main_data.dom_color, "RED");
                } else if (main_data.g > main_data.r && main_data.g > main_data.b) {
                    rgb_green(&rgb_leds);
                    strcpy(main_data.dom_color, "GREEN");
                } else {
                    rgb_blue(&rgb_leds);
                    strcpy(main_data.dom_color, "BLUE");
                }

                display_measurements();

                break;

            case NORMAL_MODE:
                green(&leds);
                flags = 0;

                if (!rgb_timer_running) {
                    rgb_timer_running = true;
                    k_timer_start(&rgb_timer, K_MSEC(RGB_TIMER_PERIOD_MS), K_MSEC(RGB_TIMER_PERIOD_MS));
                }

                get_measurements();

                /* Check all measurements with min/max */
                check_min_max(&main_data.temp, TEMP_MIN, TEMP_MAX, &flags, FLAG_TEMP);
                check_min_max(&main_data.hum, HUM_MIN, HUM_MAX, &flags, FLAG_HUM);
                check_min_max(&main_data.light, LIGHT_MIN, LIGHT_MAX, &flags, FLAG_LIGHT);
                check_min_max(&main_data.moisture, MOISTURE_MIN, MOISTURE_MAX, &flags, FLAG_MOISTURE);

                check_min_max(&main_data.c, COLOR_CLEAR_MIN, COLOR_CLEAR_MAX, &flags, FLAG_COLOR);
                check_min_max(&main_data.r, RED_MIN, RED_MAX, &flags, FLAG_COLOR);
                check_min_max(&main_data.g, GREEN_MIN, GREEN_MAX, &flags, FLAG_COLOR);
                check_min_max(&main_data.b, BLUE_MIN, BLUE_MAX, &flags, FLAG_COLOR);

                check_min_max(&main_data.x_axis, ACCEL_MIN*9.8f, ACCEL_MAX*9.8f, &flags, FLAG_ACCEL);
                check_min_max(&main_data.y_axis, ACCEL_MIN*9.8f, ACCEL_MAX*9.8f, &flags, FLAG_ACCEL);
                check_min_max(&main_data.z_axis, ACCEL_MIN*9.8f, ACCEL_MAX*9.8f, &flags, FLAG_ACCEL);

                atomic_set(&main_data.rgb_flags, flags);
                
                display_measurements();
                
                break;

            case ADVANCED_MODE:
                red(&leds);

                if (rgb_timer_running) {
                    rgb_timer_running = false;
                    k_timer_stop(&rgb_timer);
                    rgb_led_off(&rgb_leds);
                }

                k_sleep(K_MSEC(1000));
                break;
        }
    }
}
