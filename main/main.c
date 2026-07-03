/**
 * @file    main.c
 * @brief   SOLARIS combined test driver — ultrasonic + phototransistor
 *
 * Ultrasonic sensors run on ADC1 (via CD4052BE), phototransistors on ADC2
 * (via CD4051B). Different ADC units + different GPIOs, so they coexist.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "solaris_ultrasonic.h"
#include "solaris_phototransistor.h"

static const char *TAG = "SOLARIS";

#define LOOP_DELAY_MS  250

void app_main(void)
{
    /* ---- Ultrasonic: CD4052BE 4-ch mux on ADC1 ---- */
    solaris_us_config_t us_cfg = SOLARIS_US_CONFIG_DEFAULT();
    us_cfg.num_sensors = 4;                 /* adjust to match your wiring */

    solaris_us_handle_t sonar = NULL;
    bool us_ok = (solaris_us_init(&us_cfg, &sonar) == ESP_OK);
    if (!us_ok) {
        ESP_LOGE(TAG, "Ultrasonic init failed - skipping that subsystem");
    }

    /* ---- Phototransistors: CD4051B 8-ch mux on ADC2 ---- */
    /* Set your real mux select lines in SOLARIS_PT_CONFIG_DEFAULT() or here. */
    solaris_pt_config_t pt_cfg = SOLARIS_PT_CONFIG_DEFAULT();  /* 8 sensors, ch 0-7 */

    solaris_pt_handle_t pt = NULL;
    bool pt_ok = (solaris_pt_init(&pt_cfg, &pt) == ESP_OK);
    if (!pt_ok) {
        ESP_LOGE(TAG, "Phototransistor init failed - skipping that subsystem");
    }

    if (!us_ok && !pt_ok) {
        ESP_LOGE(TAG, "Both subsystems failed to init. Halting.");
        return;
    }

    /* ---- Working buffers ---- */
    solaris_us_result_t us_res[SOLARIS_US_MAX_SENSORS];
    solaris_pt_result_t pt_res[SOLARIS_PT_MAX_SENSORS];

    while (1) {
        printf("\033[2J\033[H");   /* clear terminal each pass */

        if (us_ok) {
            ESP_LOGI(TAG, "--- Ultrasonic ---");
            if (solaris_us_read(sonar, us_res) == ESP_OK) {
                solaris_us_log(sonar, us_res);
                solaris_us_print_teleplot(sonar, us_res);
            } else {
                ESP_LOGW(TAG, "Ultrasonic read failed this cycle");
            }
        }

        if (pt_ok) {
            ESP_LOGI(TAG, "--- Phototransistors ---");
            if (solaris_pt_read(pt, pt_res) == ESP_OK) {
                solaris_pt_log(pt, pt_res);
            } else {
                ESP_LOGW(TAG, "Phototransistor read failed this cycle");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    /* Not reached in this loop, but for completeness:
     * solaris_pt_deinit(pt);
     * solaris_us_deinit(sonar);
     */
}