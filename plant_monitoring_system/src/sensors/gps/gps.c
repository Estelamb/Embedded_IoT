/**
 * @file gps.c
 * @brief GPS UART handling and GGA parsing.
 *
 * ISR reads bytes from UART, accumulates lines and parses GGA/GNGGA frames.
 * When a GGA frame is parsed successfully, the parsed data is stored and a
 * semaphore is given so other threads can read it using gps_wait_for_gga().
 *
 * This implementation is intentionally simple and robust for embedded use.
 */

#include "gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>

#define BUF_SIZE 128
#define MAX_FIELDS 16

static const struct device *uart_dev = NULL;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

/* Internal parsed data storage + sync */
static gps_data_t parsed_data;
static struct k_sem parsed_sem;

/* Helper: convert NMEA lat/lon "DDMM.MMMM" or "DDDMM.MMMM" + dir to degrees decimal */
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;

    /* parse integer part before decimal point to simplify (robust for embedded) */
    float value = 0.0f;
    float decimal = 0.0f;
    bool seen_dot = false;
    float divisor = 10.0f;

    for (int i = 0; nmea[i]; i++) {
        char c = nmea[i];
        if (c >= '0' && c <= '9') {
            if (!seen_dot) {
                value = value * 10.0f + (c - '0');
            } else {
                decimal += (c - '0') / divisor;
                divisor *= 10.0f;
            }
        } else if (c == '.') {
            seen_dot = true;
        } else {
            break;
        }
    }

    value += decimal;

    /* separate degrees and minutes */
    int deg_len = (dir == 'N' || dir == 'S') ? 2 : 3; /* lat uses 2 digits degrees, lon 3 */
    int degrees = (int)(value / 100.0f);
    float minutes = value - (degrees * 100.0f);
    float result = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

/* Simple GGA parser: extracts fields from a GGA line and fills gps_data_t.
 * Returns true if parsing succeeded.
 */
static bool parse_gga(const char *line, gps_data_t *out)
{
    /* Make a mutable copy */
    char buf[BUF_SIZE];
    strncpy(buf, line, BUF_SIZE - 1);
    buf[BUF_SIZE - 1] = '\0';

    /* Tokenize by comma */
    char *fields[MAX_FIELDS] = {0};
    char *p = buf;
    int idx = 0;

    fields[idx++] = p;
    while (*p && idx < MAX_FIELDS) {
        if (*p == ',') {
            *p = '\0';
            fields[idx++] = p + 1;
        }
        p++;
    }

    /* GGA layout (indices):
     * 0 = $GPGGA
     * 1 = UTC time hhmmss.ss
     * 2 = lat DDMM.MMMM
     * 3 = N/S
     * 4 = lon DDDMM.MMMM
     * 5 = E/W
     * 6 = fix quality
     * 7 = num satellites
     * 8 = HDOP
     * 9 = altitude
     */
    if (!fields[0]) return false;
    if (!strstr(fields[0], "GGA")) return false;

    if (!fields[2] || !fields[3] || !fields[4] || !fields[5]) return false;

    float lat = nmea_to_degrees(fields[2], fields[3][0]);
    float lon = nmea_to_degrees(fields[4], fields[5][0]);

    out->lat = lat;
    out->lon = lon;

    if (fields[9]) {
        out->alt = (float)atof(fields[9]); /* meters */
    } else {
        out->alt = 0.0f;
    }

    out->sats = fields[7] ? atoi(fields[7]) : 0;
    out->hdop = fields[8] ? (float)atof(fields[8]) : 0.0f;

    if (fields[1]) {
        strncpy(out->utc_time, fields[1], sizeof(out->utc_time) - 1);
        out->utc_time[sizeof(out->utc_time) - 1] = '\0';
    } else {
        out->utc_time[0] = '\0';
    }

    return true;
}

/* UART ISR: read bytes, accumulate lines, parse GGA lines and publish parsed_data */
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '$') {
                /* start of new sentence */
                line_pos = 0;
                nmea_line[line_pos++] = (char)c;
            } else {
                if (line_pos < (BUF_SIZE - 1)) {
                    nmea_line[line_pos++] = (char)c;
                }
            }

            if (c == '\n') {
                nmea_line[line_pos] = '\0';
                /* Only parse GGA/GNGGA messages */
                if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                    gps_data_t tmp;
                    if (parse_gga(nmea_line, &tmp)) {
                        /* publish */
                        memcpy(&parsed_data, &tmp, sizeof(gps_data_t));
                        k_sem_give(&parsed_sem);
                    }
                }
                line_pos = 0;
            }
        } else {
            break;
        }
    }
}

int gps_init(const struct gps_config *cfg)
{
    if (!cfg || !cfg->dev) {
        printk("gps_init: invalid config\n");
        return -EINVAL;
    }

    uart_dev = cfg->dev;

    if (!device_is_ready(uart_dev)) {
        printk("GPS UART device not ready\n");
        return -ENODEV;
    }

    k_sem_init(&parsed_sem, 0, 1);

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    printk("GPS UART initialized\n");
    return 0;
}

int gps_wait_for_gga(gps_data_t *out, k_timeout_t timeout)
{
    if (!out) return -EINVAL;

    int ret = k_sem_take(&parsed_sem, timeout);
    if (ret < 0) return ret;

    /* copy parsed data out */
    memcpy(out, &parsed_data, sizeof(gps_data_t));
    return 0;
}
