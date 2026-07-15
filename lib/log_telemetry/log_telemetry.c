#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/time.h>
#include <solaris_telemetry.h>
#include <encoders.h>
#include "driver/temperature_sensor.h"
#include <solaris_ina228.h>
#include <stdlib.h>
#include "shared_resources.h"
#include <solaris_ina228.h>

#define NVS_NS "solaris_tel"
#define NVS_KEY_CNT "count"
#define NVS_KEY_DATA "records"

#define TAG "TELEMETRY LOG"

static temperature_sensor_handle_t temp_handle = NULL;
static temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);

// TODO get ina228 handle from pvparameters
void log_telemetry(void *pvParameters)
{
    nvs_handle_t handle;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(60000);
    int32_t count = -1;
    solaris_telemetry_t record;
    xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY); // Wait for time sync to happen before allowing logging
    solaris_ina228_result_t result;
    double sum = 0;
    double avg = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Starting logging task");
        struct timeval tv;
        gettimeofday(&tv, NULL);
        record.timestamp = tv.tv_sec;
        // TODO get implementation to get the telemetry information
        uint8_t battery_level = 0, net_power_w = 0;

        // Get state of charge of battery from the energy monitor unit
        solaris_ina228_read(ina228_handle, &result);
        if (result.current_ma == 0) // If no current is flowing then charge controller found the battery is fully charged.
        {
            battery_level = (uint8_t)100;
        }
        else
        {
            battery_level = (uint8_t)result.soc_percent;
        }

        // Get the average net power gain / loss from last minute
        xSemaphoreTake(solaris_energy_monitor_resource_with_moves, portMAX_DELAY);
        for (int i = 0; i < 60; i++)
        {
            sum += solaris_power_buffer_with_moves_included[i];
        }
        xSemaphoreGive(solaris_energy_monitor_resource_with_moves);
        avg = sum / 60;
        net_power_w = avg;

        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

        float tsens_out;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));

        ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

        record.cpu_temp = (uint8_t)tsens_out;

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

        // Do this task every 60 seconds
        vTaskDelayUntil(&last, period);
    }
}

void cpu_temp_init()
{
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}
