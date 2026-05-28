#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "solaris_icm20948.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting SOLARIS 9-DoF IMU Test...");

    // 1. Load the default configuration (SCL:40, SDA:41, INT:39)
    solaris_icm20948_config_t imu_cfg = SOLARIS_ICM20948_CONFIG_DEFAULT();
    solaris_icm20948_handle_t imu_handle = NULL;

    // 2. Initialize the sensor (Handles I2C setup, Wake, and AK09916 Bypass)
    esp_err_t err = solaris_icm20948_init(&imu_cfg, &imu_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMU. Halting.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    solaris_icm20948_result_t imu_data;

    // 3. Main processing loop
    while (1) {
        // Ignore the INT wire and force an I2C read
        esp_err_t status = solaris_icm20948_read(imu_handle, &imu_data);
        
        if (status == ESP_OK) {
            solaris_icm20948_log(&imu_data);
        } else {
            ESP_LOGW(TAG, "I2C read failed during polling");
        }
        
        // Wait 100ms before asking again (10Hz refresh rate)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}