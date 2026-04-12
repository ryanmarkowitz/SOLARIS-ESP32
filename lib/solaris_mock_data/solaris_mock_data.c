/*
Purpose of this file is to create some mock data and save it in flash.
Records are anchored to the current system time (set via BLE time sync) and
span four overlapping demo windows at increasing density toward the present:

  Month  (30d → 7d ago):  1 record per 4 h  → ~138 records
  Week   ( 7d → 1d ago):  1 record per 2 h  →  ~72 records
  Day    ( 1d → 1h ago):  1 record per 30 m →  ~46 records
  Hour   ( 1h → now   ):  1 record per 5 m  →  ~12 records
                                          Total ~268 records
*/

#include "solaris_mock_data.h"
#include "solaris_telemetry.h"
#include "solaris_common.h"
#include <math.h>
#include <time.h>

#define TAG "MOCK_DATA"

#define NVS_NS       "solaris_tel"
#define NVS_KEY_CNT  "count"
#define NVS_KEY_DATA "records"

/* Window boundaries (seconds before now) */
#define WINDOW_MONTH (30 * 24 * 3600)
#define WINDOW_WEEK  ( 7 * 24 * 3600)
#define WINDOW_DAY   ( 1 * 24 * 3600)
#define WINDOW_HOUR  (          3600)

/* Record intervals per window */
#define INTERVAL_MONTH (4 * 3600)
#define INTERVAL_WEEK  (2 * 3600)
#define INTERVAL_DAY   (   30 * 60)
#define INTERVAL_HOUR  (    5 * 60)

/* Approximate worst-case total — sized for the NVS blob */
#define MOCK_MAX_RECORDS 300

/* Solar power curve: peaks at solar noon (W), negative at night */
static int32_t solar_power_at(uint32_t ts)
{
    /* seconds into the UTC day → normalised 0..1 */
    float day_frac = (float)(ts % 86400) / 86400.0f;
    /* sine peaking at midday (0.5), zero at dawn/dusk, negative at night */
    float power = sinf((day_frac - 0.25f) * 2.0f * (float)M_PI);
    return (int32_t)(power * 600.0f); /* ±600 W range */
}

void solaris_mock_data_seed(void)
{
    static solaris_telemetry_t records[MOCK_MAX_RECORDS];
    int count = 0;

    uint32_t now = (uint32_t)time(NULL);

    /* Generate timestamps oldest→newest across four windows */
    struct { uint32_t start; uint32_t end; uint32_t interval; } windows[] = {
        { now - WINDOW_MONTH, now - WINDOW_WEEK, INTERVAL_MONTH },
        { now - WINDOW_WEEK,  now - WINDOW_DAY,  INTERVAL_WEEK  },
        { now - WINDOW_DAY,   now - WINDOW_HOUR, INTERVAL_DAY   },
        { now - WINDOW_HOUR,  now,               INTERVAL_HOUR  },
    };

    uint32_t dist_m = 0;

    for (int w = 0; w < 4; w++)
    {
        for (uint32_t ts = windows[w].start;
             ts < windows[w].end && count < MOCK_MAX_RECORDS;
             ts += windows[w].interval)
        {
            /* Battery drains during use and charges when solar is positive */
            int32_t power = solar_power_at(ts);
            /* Clamp battery 20–95 % with a slow drift tied to record index */
            uint8_t batt = (uint8_t)(57 + 38 * sinf((float)count * 0.07f));

            dist_m += (uint32_t)(windows[w].interval / 4); /* ~1 m/s average */

            records[count].timestamp       = ts;
            records[count].battery_percent = batt;
            records[count].distance_m      = dist_m;
            records[count].net_power_gain_w = power;
            count++;
        }
    }

    ESP_LOGI(TAG, "generated %d mock records anchored to t=%lu", count, (unsigned long)now);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(handle, NVS_KEY_CNT, count);
    if (err != ESP_OK)
        goto done;

    err = nvs_set_blob(handle, NVS_KEY_DATA, records,
                       (size_t)count * sizeof(solaris_telemetry_t));
    if (err != ESP_OK)
        goto done;

    err = nvs_commit(handle);

done:
    if (err != ESP_OK)
        ESP_LOGE(TAG, "failed to seed mock data: %s", esp_err_to_name(err));
    else
        ESP_LOGI(TAG, "seeded %d mock telemetry records to NVS", count);

    nvs_close(handle);
}
