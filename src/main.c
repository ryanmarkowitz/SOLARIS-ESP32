#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const gpio_num_t led_pin = GPIO_NUM_4;
static const uint32_t blink_delay_ms = 500;


static void blink_pin(gpio_num_t pin)
{
    uint8_t led_state = 0;

    // set direction of the pin to output
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);



    while (1)
    {
        led_state = !led_state;
        gpio_set_level(pin, led_state);
        
        // Delay
        vTaskDelay(blink_delay_ms/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Try ONE pin at a time.
    // If nothing happens, change this pin and re-upload.

    printf("Blink test on GPIO %d\n", (int)led_pin);
    blink_pin(led_pin);
}
