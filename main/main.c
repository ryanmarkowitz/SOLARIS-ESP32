#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "solaris_ina228.h"
#include "solaris_icm20948.h"

static const char *TAG = "MAIN_APP";

void app_main(void)
{
    solaris_ina228_config_t ina_cfg = SOLARIS_INA228_CONFIG_DEFAULT();
    solaris_icm20948_config_t imu_cfg = SOLARIS_ICM20948_CONFIG_DEFAULT();
    // 1. Centralized I2C Bus Initialization (Do this ONCE)
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 41,
        .scl_io_num = 40,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // Initialize Sensors
    solaris_ina228_handle_t ina_h = NULL;
    solaris_icm20948_handle_t imu_h = NULL;

    if (solaris_icm20948_init(&imu_cfg, &imu_h) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ICM-20948");
    }

    if (solaris_ina228_init(&ina_cfg, &ina_h) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init INA228");
    }
    // Main loop
    while (1) {
        if (ina_h) {
            solaris_ina228_result_t pwr;
            if (solaris_ina228_read(ina_h, &pwr) == ESP_OK) {
                solaris_ina228_log(ina_h, &pwr);
            }
        }

        if (imu_h) {
            solaris_icm20948_result_t imu;
            if (solaris_icm20948_read(imu_h, &imu) == ESP_OK) {
                solaris_icm20948_log(&imu);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Sample at 2Hz
    }
}