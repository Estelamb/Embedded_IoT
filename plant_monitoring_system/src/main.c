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

/* --- Limits (raw units matching your atomic storage) ----------------------- */

#define TEMP_MIN   (-10)   /**< Min temp. -10.0 C */
#define TEMP_MAX   (50)    /**< Max temp.  50.0 C */

#define HUM_MIN    (25)    /**< Min humidity 25.0 % */
#define HUM_MAX    (75)    /**< Max humidity 75.0 % */

#define LIGHT_MIN  (0)     /**< Min brightness 0.0 %  */
#define LIGHT_MAX  (100)   /**< Max brightness 100.0 % */

#define MOISTURE_MIN   (0)    /**< Min moisture 0.0 % */
#define MOISTURE_MAX   (100)  /**< Max moisture 100.0 % */

#define COLOR_CLEAR_MIN   (1)    /**< Min color clear channel raw value (dark threshold) */
#define COLOR_CLEAR_MAX   (5000)  /**< Max color clear channel raw value (saturation threshold) */
#define RED_MIN     (0)    /**< Min red channel raw value */
#define RED_MAX     (5000)  /**< Max red channel raw value */
#define GREEN_MIN   (0)    /**< Min green channel raw value */
#define GREEN_MAX   (5000)  /**< Max green channel raw value */
#define BLUE_MIN    (0)    /**< Min blue channel raw value */
#define BLUE_MAX    (5000)  /**< Max blue channel raw value */

#define ACCEL_MIN  (-2)    /**< Min accel. -2.00 g */
#define ACCEL_MAX  (2)     /**< Max accel.  2.00 g */

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

/** Timer for RGB NORMAL MODE */
static struct k_timer rgb_timer;
static atomic_t rgb_flags = ATOMIC_INIT(0);

/**
 * @brief Callback timer RGB every 0.5s.
 */
static void rgb_timer_handler(struct k_timer *timer)
{
    static uint8_t color_index = 0;
    system_mode_t mode = atomic_get(&ctx.mode);
    uint32_t flags = atomic_get(&rgb_flags);

    if (mode != NORMAL_MODE) {
        rgb_led_off(&rgb_leds);
        color_index = 0;
        return;
    }

    if (flags == 0) {
        rgb_led_off(&rgb_leds);
        color_index = 0;
        return;
    }

    /* Construimos una lista de colores activos según flags */
    uint8_t colors[8];
    uint8_t count = 0;

    if (flags & FLAG_TEMP)      colors[count++] = 0; // rojo
    if (flags & FLAG_HUM)       colors[count++] = 1; // azul
    if (flags & FLAG_LIGHT)     colors[count++] = 2; // verde
    if (flags & FLAG_MOISTURE)  colors[count++] = 3; // cian
    if (flags & FLAG_COLOR)     colors[count++] = 4; // blanco
    if (flags & FLAG_ACCEL)     colors[count++] = 5; // amarillo

    if (count == 0) {
        rgb_led_off(&rgb_leds);
        color_index = 0;
        return;
    }

    /* Selecciona el color actual */
    uint8_t color = colors[color_index % count];
    color_index++;

    rgb_led_off(&rgb_leds); // limpiar estado previo

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

    /* Inicializar timer RGB */
    k_timer_init(&rgb_timer, rgb_timer_handler, NULL);
    k_timer_start(&rgb_timer, K_MSEC(500), K_MSEC(500)); // Callback cada 0.5s

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
                    blue(&leds);

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

                    flags = 0;

                    /* Temperature */
                    if (temp < TEMP_MIN) {
                        temp = TEMP_MIN;
                        flags |= FLAG_TEMP;                    
                    } else if (temp > TEMP_MAX) {
                        temp = TEMP_MAX;
                        flags |= FLAG_TEMP;
                    }
                
                    /* Humidity */
                    if (hum < HUM_MIN) {
                        hum = HUM_MIN;
                        flags |= FLAG_HUM;
                    } else if (hum > HUM_MAX) {
                        hum = HUM_MAX;
                        flags |= FLAG_HUM;
                    }
                
                    /* Ambient light (0..100) */
                    if (light < LIGHT_MIN) {
                        light = LIGHT_MIN;
                        flags |= FLAG_LIGHT;
                    } else if (light > LIGHT_MAX) {
                        light = LIGHT_MAX;
                        flags |= FLAG_LIGHT;
                    }
                
                    /* Soil moisture (0..100) */
                    if (moisture < MOISTURE_MIN) {
                        moisture = MOISTURE_MIN;
                        flags |= FLAG_MOISTURE;
                    } else if (moisture > MOISTURE_MAX) {
                        moisture = MOISTURE_MAX;
                        flags |= FLAG_MOISTURE;
                    }
                
                    /* Color sensor (clear channel out of expected range) */
                    if (c < COLOR_CLEAR_MIN) {
                        c = COLOR_CLEAR_MIN;
                        flags |= FLAG_COLOR;
                    } else if (c > COLOR_CLEAR_MAX) {
                        c = COLOR_CLEAR_MAX;
                        flags |= FLAG_COLOR;
                    }

                    /* Color sensor (R, G, B channels out of expected range) */
                    if (r < RED_MIN) {
                        r = RED_MIN;
                        flags |= FLAG_COLOR;
                    } else if (r > RED_MAX) {
                        r = RED_MAX;
                        flags |= FLAG_COLOR;
                    }

                    if (g < GREEN_MIN) {
                        g = GREEN_MIN;
                        flags |= FLAG_COLOR;
                    } else if (g > GREEN_MAX) {
                        g = GREEN_MAX;
                        flags |= FLAG_COLOR;
                    }

                    if (b < BLUE_MIN) {
                        b = BLUE_MIN;
                        flags |= FLAG_COLOR;
                    } else if (b > BLUE_MAX) {
                        b = BLUE_MAX;
                        flags |= FLAG_COLOR;
                    }
                
                    /* Acceleration: check magnitude on any axis */
                    if (x_axis < (ACCEL_MIN*9.8f)) {
                        x_axis = (ACCEL_MIN*9.8f);
                        flags |= FLAG_ACCEL;
                    } else if (x_axis > (ACCEL_MAX*9.8f)) {
                        x_axis = (ACCEL_MAX*9.8f);
                        flags |= FLAG_ACCEL;
                    }

                    if (y_axis < (ACCEL_MIN*9.8f)) {
                        y_axis = (ACCEL_MIN*9.8f);
                        flags |= FLAG_ACCEL;
                    } else if (y_axis > (ACCEL_MAX*9.8f)) {
                        y_axis = (ACCEL_MAX*9.8f);
                        flags |= FLAG_ACCEL;
                    }

                    if (z_axis < (ACCEL_MIN*9.8f)) {
                        z_axis = (ACCEL_MIN*9.8f);
                        flags |= FLAG_ACCEL;
                    } else if (z_axis > (ACCEL_MAX*9.8f)) {
                        z_axis = (ACCEL_MAX*9.8f);
                        flags |= FLAG_ACCEL;
                    }

                    atomic_set(&rgb_flags, flags);
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
