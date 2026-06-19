#include "solaris_common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "solaris_mock_data.h"
#include "nimble_init.h"
#include "esp_pm.h"

#define TAG "NIMBLE_INIT"

/* ble_store_config_init has no public header in ESP-IDF — forward declare it */
void ble_store_config_init(void);

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason)
{
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void)
{
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void)
{
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = solaris_gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param)
{
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

void nimble_init(void)
{
    /* Local variables */
    int rc = 0;
    esp_err_t ret;

/* NimBLE stack initialization */
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 40,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_LOGI(TAG, "automatic light sleep configured");
#endif

    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }
#endif

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    xTaskCreatePinnedToCore(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL, 0);
    return;
}
