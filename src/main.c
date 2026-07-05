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

#define MODE_QUEUE_LENGTH 10

#define TAG "Main"

SemaphoreHandle_t actuator_mutex = NULL;
QueueHandle_t xModeQueue = NULL;
TaskHandle_t xSolarTracking = NULL;
TaskHandle_t xDriverFunction = NULL;

solaris_pt_config_t pt_cfg = SOLARIS_PT_CONFIG_DEFAULT();
solaris_pt_handle_t pt = NULL;

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

    // initialize the phototransistors
    bool pt_ok = (solaris_pt_init(&pt_cfg, &pt) == ESP_OK);
    if (!pt_ok)
    {
        ESP_LOGE(TAG, "Phototransistor init failed - skipping that subsystem");
    }

    actuator_mutex = xSemaphoreCreateMutex();

    xModeQueue = xQueueCreate(MODE_QUEUE_LENGTH, sizeof(solaris_mode_t));

    // IF YOU GET STACK OVERFLOW ERRORS CHANGE 4096 TO HIGHER NUMBER AS THIS IS THE STACK DEPTH ALLOCATION
    xTaskCreate(solar_tracking, "solar tracking", 4096, NULL, 13, &xSolarTracking);
    xTaskCreate(driver_function, "driving function", 4096, NULL, 10, &xDriverFunction);
    // xTaskCreate(test_motor, "test drive", 4096, NULL, 8, NULL);
}