/*
Purpose of this file is to create some mock data and save it in flash
*/

#include "solaris_mock_data.h"
#include "solaris_telemetry.h"
#include "solaris_common.h"

#define TAG "MOCK_DATA"
#define MOCK_COUNT 50
#define MOCK_BASE_TS 1743811200UL /* 2025-04-05 00:00:00 UTC */

#define NVS_NS "solaris_tel"
#define NVS_KEY_CNT "count"
#define NVS_KEY_DATA "records"

void solaris_mock_data_seed(void)
{
    solaris_telemetry_t records[MOCK_COUNT];

    // create mock data
    for (int i = 0; i < MOCK_COUNT; i++)
    {
        records[i].timestamp = MOCK_BASE_TS + (uint32_t)(i * 60);
        records[i].battery_percent = (uint8_t)(90 - i / 5);
        records[i].distance_m = (uint32_t)(i * 500);
        records[i].net_power_gain_w = 400 - i * 8;
    }

    // prepare to send mock data to flash
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(handle, NVS_KEY_CNT, MOCK_COUNT);
    if (err != ESP_OK)
        goto done;

    err = nvs_set_blob(handle, NVS_KEY_DATA, records, sizeof(records));
    if (err != ESP_OK)
        goto done;

    err = nvs_commit(handle);

done:
    if (err != ESP_OK)
        ESP_LOGE(TAG, "failed to seed mock data: %s", esp_err_to_name(err));
    else
        ESP_LOGI(TAG, "seeded %d mock telemetry records to NVS", MOCK_COUNT);

    nvs_close(handle);
}
