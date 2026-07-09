#include <stdio.h>
#include <nimble_init.h>
#include <encoders.h>
#include <motor_driver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <solaris_ina228.h>
#include <solaris_icm20948.h>
#include "esp_log.h"
#include <solaris_ultrasonic.h>
#include <solar_tracking.h>
#include "shared_resources.h"
#include <solaris_mode.h>
#include <driver.h>
#include <solaris_ina228.h>

#define EVENT_QUEUE_LENGTH 10

#define TAG "Main"

SemaphoreHandle_t actuator_mutex = NULL;
SemaphoreHandle_t solaris_energy_monitor_resource = NULL;
SemaphoreHandle_t i2c_bus_mutex = NULL;
SemaphoreHandle_t pt_bus_mutex = NULL;
QueueHandle_t xEventQueue = NULL;
TaskHandle_t xSolarTracking = NULL;
TaskHandle_t xDriverFunction = NULL;
TaskHandle_t xMoveDecision = NULL;
TaskHandle_t xImuDrive = NULL;
TaskHandle_t xImuAlign = NULL;
TaskHandle_t xUltrasonic = NULL;
TaskHandle_t xImuCollision = NULL;

solaris_pt_config_t pt_cfg = SOLARIS_PT_CONFIG_DEFAULT();
solaris_pt_handle_t pt = NULL;

solaris_ina228_config_t ina228_cfg = SOLARIS_INA228_CONFIG_DEFAULT();
solaris_ina228_handle_t ina228_handle = NULL;

solaris_icm20948_config_t imu_cfg = SOLARIS_ICM20948_CONFIG_DEFAULT();
solaris_icm20948_handle_t imu_handle = NULL;

solaris_us_config_t us_cfg = SOLARIS_US_CONFIG_DEFAULT();
solaris_us_handle_t us_handle = NULL;

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

    // Created before any I2C device init below -- INA228/ICM20948 init
    // themselves talk over I2C_NUM_0 and need this mutex to already exist.
    i2c_bus_mutex = xSemaphoreCreateMutex();
    pt_bus_mutex = xSemaphoreCreateMutex();

    // initialize the phototransistors
    bool pt_ok = (solaris_pt_init(&pt_cfg, &pt) == ESP_OK);
    if (!pt_ok)
    {
        ESP_LOGE(TAG, "Phototransistor init failed - skipping that subsystem");
    }

    // initialize the energy monitor
    if (solaris_ina228_init(&ina228_cfg, &ina228_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "INA228 init failed - skipping that subsystem");
    }

    // initialize the IMU
    if (solaris_icm20948_init(&imu_cfg, &imu_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "ICM20948 init failed - skipping that subsystem");
    }

    // initialize the ultrasonic sensors
    if (solaris_us_init(&us_cfg, &us_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Ultrasonic init failed - skipping that subsystem");
    }

    actuator_mutex = xSemaphoreCreateMutex();
    solaris_energy_monitor_resource = xSemaphoreCreateMutex();

    xEventQueue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(solaris_event_t));

    // IF YOU GET STACK OVERFLOW ERRORS CHANGE 4096 TO HIGHER NUMBER AS THIS IS THE STACK DEPTH ALLOCATION
    xTaskCreatePinnedToCore(solar_tracking, "solar tracking", 4096, NULL, 13, &xSolarTracking, 1);
    xTaskCreatePinnedToCore(driver_function, "driving function", 4096, NULL, 5, &xDriverFunction, 1);
    xTaskCreatePinnedToCore(solaris_ina228_make_move_decision, "move decision function", 4096, NULL, 8, &xMoveDecision, 0);
    xTaskCreatePinnedToCore(solaris_ina228_1s_read, "energy read", 4096, ina228_handle, 7, NULL, 0);
    xTaskCreatePinnedToCore(imu_drive_task, "imu drive", 4096, NULL, 10, &xImuDrive, 1);
    xTaskCreatePinnedToCore(imu_align_task, "imu align", 4096, NULL, 10, &xImuAlign, 1);
    xTaskCreatePinnedToCore(ultrasonic_task, "ultrasonic", 4096, NULL, 15, &xUltrasonic, 1);
    xTaskCreatePinnedToCore(imu_collision_task, "imu collision", 4096, NULL, 14, &xImuCollision, 0);
}