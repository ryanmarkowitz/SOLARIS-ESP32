#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <nimble_init.h>

void app_main(void)
{
    // Initialize nimBLE
    nimble_init();
}
