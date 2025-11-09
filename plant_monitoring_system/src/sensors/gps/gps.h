/**
 * @file gps.h
 * @brief Simple GPS helper: UART ISR, GGA parse and waiting API.
 *
 * This module provides:
 *  - gps_init(&cfg) to initialize the UART and ISR.
 *  - gps_wait_for_gga(data, timeout) to block until a parsed GGA is available.
 *
 * The parsed numeric values are returned as floats in gps_data_t.
 */

#ifndef GPS_H_
#define GPS_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

struct gps_config {
    const struct device *dev; /* UART device already resolved by caller */
};

typedef struct {
    float lat;    /* degrees (decimal) */
    float lon;    /* degrees (decimal) */
    float alt;    /* meters */
    int   sats;   /* number of satellites */
    float hdop;   /* horizontal dilution of precision */
    char  utc_time[16]; /* hhmmss.ss null-terminated if available */
} gps_data_t;

/* Initialize GPS UART and ISR.
 * Returns 0 on success, negative on error.
 */
int gps_init(const struct gps_config *cfg);

/* Wait for next parsed GGA. Returns 0 on success, -ETIMEDOUT on timeout.
 * Timeout examples: K_FOREVER, K_MSEC(2000), K_NO_WAIT
 * On success fills gps_data_t.
 */
int gps_wait_for_gga(gps_data_t *out, k_timeout_t timeout);

#endif /* GPS_H_ */
