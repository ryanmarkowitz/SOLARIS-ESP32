/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "solaris_weather.h"

#include <string.h>

static solaris_weather_t g_weather = {0};

static inline uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool solaris_weather_set_from_ble(const uint8_t *data, uint16_t len)
{
    if (len != SOLARIS_WEATHER_PAYLOAD_LEN)
        return false;

    const uint8_t *p = data;
    g_weather.sunrise = get_u32_le(p);
    p += 4;
    g_weather.sunset = get_u32_le(p);
    p += 4;

    for (int i = 0; i < SOLARIS_WEATHER_FORECAST_HOURS; i++)
    {
        g_weather.forecast[i].time = get_u32_le(p);
        p += 4;
        g_weather.forecast[i].cloud_cover_pct = *p++;
        g_weather.forecast[i].precip_probability_pct = *p++;
    }
    return true;
}

void solaris_weather_get(solaris_weather_t *weather_out)
{
    if (weather_out)
        *weather_out = g_weather;
}
