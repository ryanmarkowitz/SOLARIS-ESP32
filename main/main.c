/**
 * @file    main.c
 * @brief   SOLARIS INA228 Power Monitor — Example Usage
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "solaris_ina228.h"

#define LOOP_DELAY_MS  500

void app_main(void)
{
    // 1. Build config from defaults, override what you need
    solaris_ina228_config_t cfg = SOLARIS_INA228_CONFIG_DEFAULT();
    cfg.battery_capacity_mah = 5000.0f;  // set your actual battery capacity

    // 2. Init
    solaris_ina228_handle_t power_monitor;
    ESP_ERROR_CHECK(solaris_ina228_init(&cfg, &power_monitor));

    // 3. Read loop
    solaris_ina228_result_t result;

    //solaris_ina228_reset_accumulators(power_monitor);

    while (1) {
        ESP_ERROR_CHECK(solaris_ina228_read(power_monitor, &result));

        // Human-readable log
        solaris_ina228_log(power_monitor, &result);

        // Teleplot output
        solaris_ina228_print_teleplot(power_monitor, &result);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    // Cleanup (never reached in this example)
    solaris_ina228_deinit(power_monitor);
}
