#include <stdio.h>
#include <nimble_init.h>
#include <encoders.h>
#include <motor_driver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <solaris_ina228.h>
#include <solaris_icm20948.h>
#include "esp_log.h"
#include <solaris_ultrasonic.h>

#define LOOP_DELAY_MS 500

#define TAG "Main"

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

    // 1. Load the default configuration (SCL:40, SDA:41, INT:39)
    solaris_icm20948_config_t imu_cfg = SOLARIS_ICM20948_CONFIG_DEFAULT();
    solaris_icm20948_handle_t imu_handle = NULL;

    // 2. Initialize the sensor (Handles I2C setup, Wake, and AK09916 Bypass)
    err = solaris_icm20948_init(&imu_cfg, &imu_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize IMU. Halting.");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    solaris_icm20948_result_t imu_data;

    // 1. Build config from defaults, override what you need
    solaris_ina228_config_t cfg_power = SOLARIS_INA228_CONFIG_DEFAULT();
    cfg_power.battery_capacity_mah = 5000.0f; // set your actual battery capacity

    // 2. Init
    solaris_ina228_handle_t power_monitor;
    ESP_ERROR_CHECK(solaris_ina228_init(&cfg_power, &power_monitor));

    // 3. Read loop
    solaris_ina228_result_t result;

    // 1. Build config (start from defaults, override what you need)
    solaris_us_config_t cfg_ultrasonic = SOLARIS_US_CONFIG_DEFAULT();
    cfg_ultrasonic.num_sensors = 4; // adjust to match your wiring

    // 2. Init
    solaris_us_handle_t sonar;
    ESP_ERROR_CHECK(solaris_us_init(&cfg_ultrasonic, &sonar));

    // 3. Working buffer
    solaris_us_result_t results[SOLARIS_US_MAX_SENSORS];

    while (1)
    {
        ESP_ERROR_CHECK(solaris_ina228_read(power_monitor, &result));

        // Human-readable log
        solaris_ina228_log(power_monitor, &result);

        // Teleplot output
        solaris_ina228_print_teleplot(power_monitor, &result);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));

        // Ignore the INT wire and force an I2C read
        esp_err_t status = solaris_icm20948_read(imu_handle, &imu_data);

        if (status == ESP_OK)
        {
            solaris_icm20948_log(&imu_data);
        }
        else
        {
            ESP_LOGW(TAG, "I2C read failed during polling");
        }

        // Wait 100ms before asking again (10Hz refresh rate)
        vTaskDelay(pdMS_TO_TICKS(100));

        // Trigger + read all sensors
        ESP_ERROR_CHECK(solaris_us_read(sonar, results));

        // Human-readable log
        solaris_us_log(sonar, results);

        // Teleplot / Serial Plotter output
        solaris_us_print_teleplot(sonar, results);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    // Cleanup (never reached in this example)
    solaris_ina228_deinit(power_monitor);
    solaris_us_deinit(sonar);
}
