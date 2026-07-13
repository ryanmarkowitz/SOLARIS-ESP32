#include "solaris_telemetry.h"
#include "solaris_common.h"
#include <string.h>

#define TAG "TELEMETRY"
#define NVS_NS "solaris_tel"
#define NVS_KEY_CNT "count"
#define NVS_KEY_DATA "records"

static solaris_telemetry_t g_telemetry = {0};

static solaris_telemetry_t g_log[SOLARIS_TELEMETRY_LOG_CAPACITY];
static int g_log_count = 0;

// read telemetry data stored in flash and store that info into g_log
void solaris_telemetry_load(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "no telemetry data in NVS (%s)", esp_err_to_name(err));
        return;
    }

    int32_t count = 0;
    nvs_get_i32(handle, NVS_KEY_CNT, &count);

    if (count > 0 && count <= SOLARIS_TELEMETRY_LOG_CAPACITY)
    {
        /* Query the actual stored blob size before reading it. If a prior
         * firmware wrote records using a different (e.g. smaller) layout
         * of solaris_telemetry_t, the blob won't match count * sizeof(...)
         * and reading it as the current struct layout would silently
         * misinterpret the bytes (nvs_get_blob only errors when the
         * caller's buffer is too small, not too large). */
        size_t stored_size = 0;
        err = nvs_get_blob(handle, NVS_KEY_DATA, NULL, &stored_size);
        size_t expected_size = (size_t)count * sizeof(solaris_telemetry_t);

        if (err == ESP_OK && stored_size == expected_size)
        {
            size_t size = expected_size;
            err = nvs_get_blob(handle, NVS_KEY_DATA, g_log, &size);
            if (err == ESP_OK)
            {
                g_log_count = (int)count;
                ESP_LOGI(TAG, "loaded %d telemetry records from NVS", g_log_count);
            }
            else
            {
                ESP_LOGE(TAG, "failed to read telemetry blob: %s", esp_err_to_name(err));
            }
        }
        else if (err == ESP_OK)
        {
            ESP_LOGW(TAG,
                     "telemetry blob size mismatch (stored %u, expected %u) - "
                     "discarding stale log from an incompatible firmware version",
                     (unsigned)stored_size, (unsigned)expected_size);
            nvs_close(handle);
            solaris_telemetry_log_clear();
            return;
        }
        else
        {
            ESP_LOGE(TAG, "failed to query telemetry blob size: %s", esp_err_to_name(err));
        }
    }

    nvs_close(handle);
}

// set our global variable g_telemetry to a passed trhough telemetry
void solaris_telemetry_set(const solaris_telemetry_t *telemetry)
{
    if (telemetry)
        g_telemetry = *telemetry;
}

// set a telemetry struct to the g_telemetry global variable
void solaris_telemetry_get(solaris_telemetry_t *telemetry_out)
{
    if (telemetry_out)
        *telemetry_out = g_telemetry;
}

// just returns the number of telemetry components
int solaris_telemetry_log_count(void)
{
    return g_log_count;
}

// get the next set of telemetry to send. We do 15 telemetry at a time
int solaris_telemetry_log_page_count(void)
{
    if (g_log_count == 0)
        return 0;
    return (g_log_count + SOLARIS_TELEMETRY_RECORDS_PER_PAGE - 1) /
           SOLARIS_TELEMETRY_RECORDS_PER_PAGE;
}

// Get the next set of telemetry data to gather
bool solaris_telemetry_log_get_page(int page, solaris_telemetry_t *records_out,
                                    int *count_out)
{
    int total_pages = solaris_telemetry_log_page_count();
    if (page < 0 || page >= total_pages || !records_out || !count_out)
        return false;

    int start = page * SOLARIS_TELEMETRY_RECORDS_PER_PAGE;
    int remaining = g_log_count - start;
    int count = remaining < SOLARIS_TELEMETRY_RECORDS_PER_PAGE
                    ? remaining
                    : SOLARIS_TELEMETRY_RECORDS_PER_PAGE;

    memcpy(records_out, &g_log[start], (size_t)count * sizeof(solaris_telemetry_t));
    *count_out = count;
    return true;
}

// Erase telemetry data
void solaris_telemetry_log_clear(void)
{
    g_log_count = 0;

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) != ESP_OK)
        return;

    nvs_erase_key(handle, NVS_KEY_CNT);
    nvs_erase_key(handle, NVS_KEY_DATA);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "telemetry log cleared");
}
