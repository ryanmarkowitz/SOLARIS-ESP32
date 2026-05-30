/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef SOLARIS_TELEMETRY_H
#define SOLARIS_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define SOLARIS_TELEMETRY_LOG_CAPACITY 1440
#define SOLARIS_TELEMETRY_RECORDS_PER_PAGE 30

typedef struct
{
    uint32_t timestamp; /* Unix timestamp */
    uint8_t cpu_temp;   /* CPU temperature in °C */
    uint8_t battery_percent;
    uint8_t distance_m;
    int8_t net_power_gain_w;
} solaris_telemetry_t;

/* Load telemetry records from NVS into memory */
void solaris_telemetry_load(void);

/* Live telemetry (current sensor readings) */
void solaris_telemetry_set(const solaris_telemetry_t *telemetry);
void solaris_telemetry_get(solaris_telemetry_t *telemetry_out);

/* Telemetry log (stored records for BLE upload) */
int solaris_telemetry_log_count(void);
int solaris_telemetry_log_page_count(void);
bool solaris_telemetry_log_get_page(int page, solaris_telemetry_t *records_out,
                                    int *count_out);
void solaris_telemetry_log_clear(void);

#endif // SOLARIS_TELEMETRY_H
