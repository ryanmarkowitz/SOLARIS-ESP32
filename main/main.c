/**
 * @file    main.c
 * @brief   SOLARIS Ultrasonic Sensor — Example Usage
 *
 * Drop the solaris_ultrasonic component into your project's components/
 * directory, then use this as your app_main.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "solaris_ultrasonic.h"

#define LOOP_DELAY_MS  1000

void app_main(void)
{
    // 1. Build config (start from defaults, override what you need)
    solaris_us_config_t cfg = SOLARIS_US_CONFIG_DEFAULT();
    cfg.num_sensors = 4;   // adjust to match your wiring

    // 2. Init
    solaris_us_handle_t sonar;
    ESP_ERROR_CHECK(solaris_us_init(&cfg, &sonar));

    // 3. Working buffer
    solaris_us_result_t results[SOLARIS_US_MAX_SENSORS];

    while (1) {
        // Trigger + read all sensors
        ESP_ERROR_CHECK(solaris_us_read(sonar, results));

        // Human-readable log
        solaris_us_log(sonar, results);

        // Teleplot / Serial Plotter output
        solaris_us_print_teleplot(sonar, results);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    // Cleanup (never reached in this example)
    solaris_us_deinit(sonar);
}