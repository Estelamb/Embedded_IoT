/**
 * @file gps.h
 * @brief Simple GPS library for parsing NMEA data from UART.
 */

#ifndef GPS_H_
#define GPS_H_

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

struct gps_config {
    const struct device *dev;
};

/** Estructura con datos GPS básicos */
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int satellites;
    float hdop;
    char time[16];
} gps_data_t;

/**
 * @brief Inicializa el GPS y configura la interrupción UART.
 *
 * @return 0 si ok, -1 si falla.
 */
int gps_init(const struct gps_config *cfg);

/**
 * @brief Obtiene los últimos datos válidos del GPS.
 *
 * @param[out] data Estructura donde se guardan los valores.
 * @return true si los datos son válidos, false si no hay nuevos datos.
 */
bool gps_get_data(gps_data_t *data);

#endif /* GPS_H_ */
