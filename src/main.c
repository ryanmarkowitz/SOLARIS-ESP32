#include <stdio.h>
#include <nimble_init.h>
#include <encoders.h>
#include <motor_driver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <solaris_ina228.h>

#define LOOP_DELAY_MS 500

void app_main(void)
{
    // Initialize nimBLE
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay
    nimble_init();

    // initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    encoder_init();
    motor_init();
    test_motor();

    // 1. Build config from defaults, override what you need
    solaris_ina228_config_t cfg = SOLARIS_INA228_CONFIG_DEFAULT();
    cfg.battery_capacity_mah = 5000.0f; // set your actual battery capacity

    // 2. Init
    solaris_ina228_handle_t power_monitor;
    ESP_ERROR_CHECK(solaris_ina228_init(&cfg, &power_monitor));

    // 3. Read loop
    solaris_ina228_result_t result;

    // solaris_ina228_reset_accumulators(power_monitor);

    while (1)
    {
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
