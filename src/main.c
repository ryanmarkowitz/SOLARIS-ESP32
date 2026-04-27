#include <stdio.h>
#include <nimble_init.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    // Initialize nimBLE
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay
    nimble_init();
}
