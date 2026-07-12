#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/time.h>
#include <solaris_telemetry.h>
#include <encoders.h>

#define NVS_NS "solaris_tel"
#define NVS_KEY_CNT "count"
#define NVS_KEY_DATA "records"

#define TAG "TELEMETRY LOG"

void log_telemetry(void *pvParameters)
{
    nvs_handle_t handle;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(60000);
    int32_t count = -1;
    solaris_telemetry_t record;
    // TODO get a task signal to start after BLE gets initial connection
    while (1)
    {

        struct timeval tv;
        gettimeofday(&tv, NULL);
        if (tv.tv_sec < 1700000000) // If the recorded year is before 2023 then the ESP likely hasn't synced yet. Don't make a new log
        {                           // Successful time of day grabbed
            record.timestamp = tv.tv_sec;
            // TODO get implementation to get the telemetry information
            uint8_t battery_level = 0, cpu_temp = 0, net_power_w = 0;
            record.distance_m = get_distance_traveled();
            record.battery_percent = battery_level;
            record.net_power_gain_w = net_power_w;

            esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
                return;
            }
            if (count == -1)
            { // initialize count if it's already saved in flash. Otherwise, we set count to 0
                err = nvs_get_i32(handle, NVS_KEY_CNT, &count);
                if (err != ESP_OK)
                {
                    count = 0;
                }
            }
            count++; // increment the count of logs as we add a new entry
            err = nvs_set_blob(handle, NVS_KEY_DATA, &record, sizeof(solaris_telemetry_t));

            if (err != ESP_OK)
                goto done;

            err = nvs_set_i32(handle, NVS_KEY_CNT, count);
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
        // Do this task every 60 seconds
        vTaskDelayUntil(&last, period);
    }
}