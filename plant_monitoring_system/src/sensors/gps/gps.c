#include "gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

#define BUF_SIZE 128

static const struct device *uart_dev;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;
static gps_data_t last_data;
static bool data_valid = false;

/* === Conversión NMEA === */
static float nmea_to_degrees(const char *nmea, char dir) {
    if (!nmea || strlen(nmea) < 4) return 0.0f;
    float value = atof(nmea);
    int degrees = (int)(value / 100);
    float minutes = value - (degrees * 100);
    float result = degrees + (minutes / 60.0f);
    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

/* === Parser de $GPGGA === */
static void parse_gpgga(char *line) {
    char *fields[15] = {0};
    int field = 0;
    char *p = strtok(line, ",");

    while (p && field < 15) {
        fields[field++] = p;
        p = strtok(NULL, ",");
    }

    if (field > 9 && fields[2] && fields[3] && fields[4] && fields[5]) {
        strncpy(last_data.time, fields[1], sizeof(last_data.time));
        last_data.latitude = nmea_to_degrees(fields[2], fields[3][0]);
        last_data.longitude = nmea_to_degrees(fields[4], fields[5][0]);
        last_data.satellites = atoi(fields[7]);
        last_data.hdop = atof(fields[8]);
        last_data.altitude = atof(fields[9]);
        data_valid = true;
    }
}

/* === Interrupción UART === */
static void uart_isr(const struct device *dev, void *user_data) {
    uint8_t c;
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '$') line_pos = 0;
            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;
                if (c == '\n') {
                    nmea_line[line_pos] = '\0';
                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                        parse_gpgga(nmea_line);
                    }
                    line_pos = 0;
                }
            }
        }
    }
}

/* === API pública === */
int gps_init(const struct gps_config *cfg) {
    if (!cfg || !cfg->dev) {
        printk("GPS: configuración inválida\n");
        return -1;
    }

    uart_dev = cfg->dev;

    if (!device_is_ready(uart_dev)) {
        printk("GPS: UART no está lista\n");
        return -1;
    }

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    printk("GPS inicializado en %s\n", uart_dev->name);
    return 0;
}

bool gps_get_data(gps_data_t *data) {
    if (!data_valid) return false;
    *data = last_data;
    data_valid = false;
    return true;
}
