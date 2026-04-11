/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef SOLARIS_WEATHER_H
#define SOLARIS_WEATHER_H

#include <stdbool.h>
#include <stdint.h>

#define SOLARIS_WEATHER_FORECAST_HOURS 24

/*
 * BLE write payload layout (152 bytes, all little-endian):
 *   [0..3]   sunrise  (uint32_t)
 *   [4..7]   sunset   (uint32_t)
 *   [8..151] 24 x { time(4) cloud_cover_pct(1) precip_probability_pct(1) }
 */
#define SOLARIS_WEATHER_PAYLOAD_LEN \
    (4 + 4 + SOLARIS_WEATHER_FORECAST_HOURS * 6)

typedef struct
{
    uint32_t time;
    uint8_t cloud_cover_pct;
    uint8_t precip_probability_pct;
} solaris_forecast_entry_t;

typedef struct
{
    uint32_t sunrise;
    uint32_t sunset;
    solaris_forecast_entry_t forecast[SOLARIS_WEATHER_FORECAST_HOURS];
} solaris_weather_t;

bool solaris_weather_set_from_ble(const uint8_t *data, uint16_t len);
void solaris_weather_get(solaris_weather_t *weather_out);

#endif // SOLARIS_WEATHER_H
