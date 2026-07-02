#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "solaris_phototransistor.h"

static const char *TAG = "SOLARIS";

void app_main(void)
{
    //change mux select lines before starting
    solaris_pt_config_t pt_cfg = SOLARIS_PT_CONFIG_DEFAULT(); 
    solaris_pt_handle_t pt = NULL;
    if (solaris_pt_init(&pt_cfg, &pt) != ESP_OK) {
        ESP_LOGE(TAG, "Phototransistor init failed");
        return;
    }

    solaris_pt_result_t pt_res[SOLARIS_PT_MAX_SENSORS];

    while (1) {
        printf("\033[2J\033[H");   /* clear terminal */

        if (solaris_pt_read(pt, pt_res) == ESP_OK) {
            solaris_pt_log(pt, pt_res);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Not reached, but for completeness:
    // solaris_pt_deinit(pt);
}