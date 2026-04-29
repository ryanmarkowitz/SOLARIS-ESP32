#include "encoders.h"
#include <motor_driver.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#define NVS_NAMESPACE "panel"
#define NVS_KEY_PAN_POSITION "pan_position"

#define TAG "ENCODER_SERVICE"

#define LOW_LIMIT -32768
#define HIGH_LIMIT 32767
#define FORWARD_TARGET 439   // maps to 30 degrees
#define BACKWARD_TARGET -449 // maps to -30 degrees
#define CHAN_GPIO_A 5
#define DIRECTION_PIN 11

pcnt_unit_handle_t pcnt_unit1 = NULL;
pcnt_channel_handle_t pcnt_chan1 = NULL;
static QueueHandle_t encoder_queue = NULL;

int saved_position = 0;

// When watchpoint is reached, send high priority task through queue sending the count in PCNT
static bool on_encoder_limit_reached(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static void encoder_handler(void *param)
{
    QueueHandle_t encoder_queue = (QueueHandle_t)param;
    int watch_point_value;

    while (1)
    {
        // wait for queue to be sent from on_encoder_limit_reached ISR
        if (xQueueReceive(encoder_queue, &watch_point_value, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Watchpoint reached: %d", watch_point_value);
            if (watch_point_value >= FORWARD_TARGET)
            {
                stop_motor();
                panel_set_limit_state(PANEL_AT_UPPER_LIMIT);
                ESP_LOGI(TAG, "Panel at upper limit");
            }
            else if (watch_point_value <= BACKWARD_TARGET)
            {
                stop_motor();
                panel_set_limit_state(PANEL_AT_LOWER_LIMIT);
                ESP_LOGI(TAG, "Panel at lower limit");
            }
        }
    }
}

// loads the previous pan position of the solar panel from flash memory
static int load_position_from_flash()
{
    nvs_handle_t nvs_handle;
    int32_t position = 0;

    esp_err_t err;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS namespace not found");
        nvs_close(nvs_handle);
        return 0;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle");
        nvs_close(nvs_handle);
        return 0;
    }

    err = nvs_get_i32(nvs_handle, NVS_KEY_PAN_POSITION, &position);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Position not found in flash");
        nvs_close(nvs_handle);
        return 0;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading position from flash");
        nvs_close(nvs_handle);
        return 0;
    }
    else
    {
        ESP_LOGI(TAG, "Position loaded from flash: %d", (int)position);
    }
    nvs_close(nvs_handle);
    return position;
}

// initialize the PCNT for the panning of panel. Max degree is 30.
void encoder_init()
{
    pcnt_unit_config_t unit_config1 = {
        .high_limit = HIGH_LIMIT,
        .low_limit = LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config1, &pcnt_unit1));

    pcnt_chan_config_t chan_config1 = {
        .edge_gpio_num = CHAN_GPIO_A,
        .level_gpio_num = DIRECTION_PIN,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit1, &chan_config1, &pcnt_chan1));

    // Increment counter on rising edge when direction pin is forward, deincrement on rising edge when direction pin is reverse
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // return previous position from flash
    saved_position = load_position_from_flash();

    // Set watchpoint endpoints to relative position rather than the counter's absolute position
    int forward_max_position = FORWARD_TARGET - saved_position;
    int backward_max_position = BACKWARD_TARGET - saved_position;

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit1, forward_max_position));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit1, backward_max_position));
    pcnt_unit_clear_count(pcnt_unit1);

    pcnt_event_callbacks_t callbacks = {
        .on_reach = on_encoder_limit_reached,
    };

    // set config to filter noise (10us. Max no load is 30RPM with 5281 PPR at the Output Shaft. Frequency is 2.64kHz max or 378us between pulses.)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit1, &filter_config));

    // Create the task for the watchpoint callback
    encoder_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit1, &callbacks, encoder_queue));

    xTaskCreate(encoder_handler, "encoder_handler", 4096, encoder_queue, 10, NULL);

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit1));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit1));
}

// when motor drivers are done moving the panel, save the position to flash
esp_err_t save_position_to_flash()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // get the absolute position of the solar panel
    int position;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit1, &position));
    int absolute_position = position + saved_position;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle");
        goto done;
    }

    // write the absolute position to flash
    err = nvs_set_i32(nvs_handle, NVS_KEY_PAN_POSITION, absolute_position);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing to NVS");
        goto done;
    }

done:
    nvs_close(nvs_handle);
    return err;
}
