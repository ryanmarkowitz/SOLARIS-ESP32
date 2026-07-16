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

#define TAG "TELEMETRY LOG"

static temperature_sensor_handle_t temp_handle = NULL;
static temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);

// TODO get ina228 handle from pvparameters
void log_telemetry(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(60000);
    solaris_telemetry_t record;
    xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY); // Wait for time sync to happen before allowing logging
    solaris_ina228_result_t result;
    double sum = 0;
    double avg = 0;
    float distance_m;
    while (1)
    {
        ESP_LOGI(TAG, "Starting logging task");
        struct timeval tv;
        gettimeofday(&tv, NULL);
        record.timestamp = tv.tv_sec;
        uint8_t battery_level = 0;
        int8_t net_power_w = 0;

        // Get state of charge of battery from the energy monitor unit
        solaris_ina228_read(ina228_handle, &result);
        battery_level = result.soc_percent;

        // Get the average net power gain / loss from last minute
        xSemaphoreTake(solaris_energy_monitor_resource_with_moves, portMAX_DELAY);
        for (int i = 0; i < 60; i++)
        {
            sum += solaris_power_buffer_with_moves_included[i];
        }
        xSemaphoreGive(solaris_energy_monitor_resource_with_moves);
        avg = sum / 60;
        if (avg > 127.0)
            avg = 127.0;
        else if (avg < -127.0)
            avg = -127.0;
        net_power_w = (int8_t)avg;

        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

        float tsens_out;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));

        ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

        record.cpu_temp = (uint8_t)tsens_out;
        record.battery_percent = battery_level;
        record.net_power_gain_w = net_power_w * 100;
        record.distance_m = get_distance_traveled();

        solaris_telemetry_set(&record);
        solaris_telemetry_log_append(&record);

        // Do this task every 60 seconds
        vTaskDelayUntil(&last, period);
    }
}

void cpu_temp_init()
{
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}
