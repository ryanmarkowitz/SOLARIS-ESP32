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
#include <solar_tracking.h>

#define LOOP_DELAY_MS 500

#define TAG "Main"

void app_main(void)
{
    // Initialize nimBLE
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay

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
    init_solar_adc();

    // IF YOU GET STACK OVERFLOW ERRORS CHANGE 4096 TO HIGHER NUMBER AS THIS IS THE STACK DEPTH ALLOCATION
    xTaskCreate(solar_tracking, "solar tracking", 4096, NULL, 15, NULL);
    // xTaskCreate(test_motor(), "test drive", 4096, NULL, 8, NULL);    UNCOMMENT AND CALL THIS FUNCTION FOR STRAIGHT LINE DEMO. MAKE SURE TO COMMENT OUT THE xTaskCreate FUNCTION ABOVE FIRST!!!
}
