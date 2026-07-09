#include "encoders.h"
#include <motor_driver.h>
#include "esp_log.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"

#define NVS_NAMESPACE "panel"
#define NVS_KEY_PAN_POSITION "pan_position"
#define NVS_KEY_TILT_POSITION "tilt_position"

#define TAG "ENCODER_SERVICE"

#define LOW_LIMIT -32768
#define HIGH_LIMIT 32767
#define HIGH_LIMIT_FR 28387
#define FORWARD_TARGET_PAN 2650   // maps to 360 degrees
#define BACKWARD_TARGET_PAN -2650 // maps to -360 degrees
#define FORWARD_TARGET_TILT 350   // maps to +30 degrees
#define BACKWARD_TARGET_TILT -350 // maps to -30 degrees

static int load_position_from_flash(uint8_t encoder_id);
static encoder_ctxt_t enc_ctxt[4];

static int forward_max_position, backward_max_position;

int saved_pan_position = 0;
int saved_tilt_position = 0;

/*
Pan encoder - Pin 30 | Dir - Pin 11
Tilt encoder - Pin 28 | Dir - Pin 31
FL encoder - Pin 35
FR encoder - Pin 26
*/

static const encoder_pins_t encoder_pins[NUM_ENCODERS] = {
    {.encoder_gpio = 37, .dir_gpio = 18},
    {.encoder_gpio = 35, .dir_gpio = 38},
    {.encoder_gpio = 42, .dir_gpio = -1},
    {.encoder_gpio = 45, .dir_gpio = -1}};

/*
ENCODER 0 - PAN ENCODER
ENCODER 1 - TILT ENCODER
ENCODER 3 - FL ENCODER
ENCODER 4 - FR ENCODER
*/
static encoder_t encoders[NUM_ENCODERS];

// used to track if driving motors overflowed and if so how many times
// before moving is stopped.
uint8_t overflow_counter_FL = 0;
uint8_t overflow_counter_FR = 0;

static QueueHandle_t encoder_queue = NULL;

int get_pulse_count(uint8_t encoder_id)
{
    int count = 0;
    pcnt_unit_get_count(encoders[encoder_id].pcnt_unit, &count);
    return count;
}

// When watchpoint is reached, send high priority task through queue sending the count in PCNT
static bool on_encoder_limit_reached(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    encoder_ctxt_t *ctx = (encoder_ctxt_t *)user_ctx;
    encoder_evt_t evt = {
        .id = ctx->id,
        .watch_point_value = edata->watch_point_value,
        .dir_level = (ctx->dir_gpio >= 0) ? gpio_get_level(ctx->dir_gpio) : -1};

    xQueueSendFromISR(ctx->queue, &evt, &high_task_wakeup);

    return (high_task_wakeup == pdTRUE);
}

static void encoder_handler(void *param)
{
    QueueHandle_t encoder_queue = (QueueHandle_t)param;
    encoder_evt_t evt;

    while (1)
    {
        // wait for queue to be sent from on_encoder_limit_reached ISR
        if (xQueueReceive(encoder_queue, &evt, portMAX_DELAY))
            ESP_LOGI(TAG, "ISR Triggered");
        ESP_LOGI(TAG, "evt id: %d", evt.id);
        ESP_LOGI(TAG, "evt dir %d", evt.dir_level);
        ESP_LOGI(TAG, "evt watch point %d", evt.watch_point_value);
        {
            switch (evt.id)
            {
            case PAN_ENCODER_ID:
                ESP_LOGI(TAG, "Watchpoint reached for pan motor: %d", evt.watch_point_value);
                if (evt.watch_point_value >= forward_max_position && (evt.dir_level == 0))
                {
                    stop_motor(MOTOR_PAN_ID);
                    panel_set_limit_state(PANEL_PAN_ID, PANEL_AT_UPPER_LIMIT);
                    ESP_LOGI(TAG, "Pan panel at upper limit");
                }
                else if (evt.watch_point_value <= backward_max_position && (evt.dir_level == 1))
                {
                    stop_motor(MOTOR_PAN_ID);
                    panel_set_limit_state(PANEL_PAN_ID, PANEL_AT_LOWER_LIMIT);
                    ESP_LOGI(TAG, "Pan panel at lower limit");
                }
                break;
            case TILT_ENCODER_ID:
                ESP_LOGI(TAG, "Watchpoint reached for tilt motor: %d", evt.watch_point_value);
                if (evt.watch_point_value >= forward_max_position && (evt.dir_level == 0))
                {
                    stop_motor(MOTOR_TILT_ID);
                    panel_set_limit_state(PANEL_TILT_ID, PANEL_AT_UPPER_LIMIT);
                    ESP_LOGI(TAG, "Tilt panel at upper limit");
                }
                else if (evt.watch_point_value <= backward_max_position && (evt.dir_level == 1))
                {
                    stop_motor(MOTOR_TILT_ID);
                    panel_set_limit_state(PANEL_TILT_ID, PANEL_AT_LOWER_LIMIT);
                    ESP_LOGI(TAG, "Tilt panel at lower limit");
                }
                break;
            case FL_ENCODER_ID:
                ESP_LOGI(TAG, "Overflow reached for FL encoder. At overflow #%d", ++overflow_counter_FL);
                break;
            case FR_ENCODER_ID:
                ESP_LOGI(TAG, "Overflow reached for FR encoder. At overflow #%d", ++overflow_counter_FR);
                break;
            }
        }
    }
}

// initialize the PCNT for the panning of panel. Max degree is 30.
void encoder_init()
{
    pcnt_unit_config_t unit_config1 = {
        .high_limit = HIGH_LIMIT,
        .low_limit = LOW_LIMIT,
    };

    pcnt_unit_config_t unit_config2 = {
        .high_limit = HIGH_LIMIT_FR,
        .low_limit = LOW_LIMIT};

    pcnt_event_callbacks_t callbacks = {
        .on_reach = on_encoder_limit_reached,
    };

    // initialzie the queue size for an encoder event
    encoder_queue = xQueueCreate(16, sizeof(encoder_evt_t));

    // configure most encoders to have the max high and low limit
    for (int i = 0; i < NUM_ENCODERS - 1; i++)
    {
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config1, &encoders[i].pcnt_unit));
    }

    // configure the FR max high limit to something different to avoid conflicting interrupts between FL encoder
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config2, &encoders[FR_ENCODER_ID].pcnt_unit));

    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        encoders[i].pins = encoder_pins[i];
        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = encoders[i].pins.encoder_gpio,
            .level_gpio_num = encoders[i].pins.dir_gpio,
        };
        ESP_ERROR_CHECK(pcnt_new_channel(encoders[i].pcnt_unit, &chan_config, &encoders[i].channel_handle));
        if (i <= 1)
        { // i = 0 or 1 indicates either pan or tilt action meaning direction is needed information
            // Increment counter on rising edge when direction pin is forward, deincrement on rising edge when direction pin is reverse

            // TODO EVENTUALLY ALL ENCODERS NEED TO WORK WITH OPTOCOUPLER. THE ELSE BLOCK WOULD BE THE RIGHT WAY TO DO IT
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(encoders[i].channel_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(encoders[i].channel_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            if (i == 0)
            {
                // return previous position from flash
                saved_pan_position = load_position_from_flash(i);

                // Set watchpoint endpoints to relative position rather than the counter's absolute position
                forward_max_position = FORWARD_TARGET_PAN - saved_pan_position;
                backward_max_position = BACKWARD_TARGET_PAN - saved_pan_position;

                if (forward_max_position <= 0)
                {
                    forward_max_position = 0;
                    panel_set_limit_state(PAN_ENCODER_ID, PANEL_AT_UPPER_LIMIT);
                }
                else if (backward_max_position >= 0)
                {
                    backward_max_position = 0;
                    panel_set_limit_state(PAN_ENCODER_ID, PANEL_AT_LOWER_LIMIT);
                }
                ESP_LOGI(TAG, "forward max position for pan: %d", forward_max_position);
                ESP_LOGI(TAG, "backward max position for pan %d", backward_max_position);
            }
            else if (i == 1)
            {
                // return previous position from flash
                saved_tilt_position = load_position_from_flash(i);

                // Set watchpoint endpoints to relative position rather than the counter's absolute position
                forward_max_position = FORWARD_TARGET_TILT - saved_tilt_position;
                backward_max_position = BACKWARD_TARGET_TILT - saved_tilt_position;

                if (forward_max_position <= 0)
                {
                    forward_max_position = 0;
                    panel_set_limit_state(TILT_ENCODER_ID, PANEL_AT_UPPER_LIMIT);
                }
                else if (backward_max_position >= 0)
                {
                    backward_max_position = 0;
                    panel_set_limit_state(TILT_ENCODER_ID, PANEL_AT_LOWER_LIMIT);
                }

                ESP_LOGI(TAG, "forward max position for tilt: %d", forward_max_position);
                ESP_LOGI(TAG, "backward max position for tilt %d", backward_max_position);
            }
            ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoders[i].pcnt_unit, forward_max_position));
            ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoders[i].pcnt_unit, backward_max_position));
        }
        else
        {
            // just increment counter on rising edge
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(encoders[i].channel_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoders[i].pcnt_unit, 0)); // When rolling the counter over, initiaite the callback
        }
        pcnt_unit_clear_count(encoders[i].pcnt_unit);
        // set config to filter noise (10us. Max no load is 30RPM with 5281 PPR at the Output Shaft. Frequency is 2.64kHz max or 378us between pulses.)
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 10000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoders[i].pcnt_unit, &filter_config));
    }

    // configure the callback
    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        enc_ctxt[i] = (encoder_ctxt_t){.id = i, .queue = encoder_queue, .dir_gpio = encoders[i].pins.dir_gpio};
        ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoders[i].pcnt_unit, &callbacks, &enc_ctxt[i]));
    }

    // create the task for the encoder handler function
    xTaskCreate(encoder_handler, "encoder_handler", 4096, encoder_queue, 18, NULL);

    // enable and start the PCNT units
    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        ESP_ERROR_CHECK(pcnt_unit_enable(encoders[i].pcnt_unit));
        ESP_ERROR_CHECK(pcnt_unit_start(encoders[i].pcnt_unit));
    }
}

// loads the previous pan position of the solar panel from flash memory
static int load_position_from_flash(uint8_t encoder_id)
{
    const char *key_string = (encoder_id == PAN_ENCODER_ID) ? NVS_KEY_PAN_POSITION : NVS_KEY_TILT_POSITION;
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

    err = nvs_get_i32(nvs_handle, key_string, &position);
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
    // return position;
    return 0;
}

// when motor drivers are done moving the panel, save the position to flash
esp_err_t save_position_to_flash(uint8_t encoder_id)
{
    // Get the correct key based off what encoder id was given
    const char *key_string = (encoder_id == PAN_ENCODER_ID) ? NVS_KEY_PAN_POSITION : NVS_KEY_TILT_POSITION;

    nvs_handle_t nvs_handle;
    esp_err_t err;

    // get the absolute position of the solar panel
    int position;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoders[encoder_id].pcnt_unit, &position));
    int saved_pan_or_tilt = (encoder_id == PAN_ENCODER_ID) ? saved_pan_position : saved_tilt_position;
    // int absolute_position = position + saved_pan_or_tilt;
    int absolute_position = 0;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle");
        goto done;
    }

    // write the absolute position to flash
    err = nvs_set_i32(nvs_handle, key_string, absolute_position);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing to NVS");
        goto done;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error committing to NVS");
    }

done:
    nvs_close(nvs_handle);
    return err;
}

int get_distance_traveled()
{
    // TODO change the pulses to convert to m traveled
    // This implementation will fail as is since there will be overflows.
    // We need to convert pulses to distance traveled before summing the distance
    // This is just temp code for idea of where to go next
    int cur_pan_pulses, cur_tilt_pulses;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoders[PAN_ENCODER_ID].pcnt_unit, &cur_pan_pulses));
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoders[TILT_ENCODER_ID].pcnt_unit, &cur_tilt_pulses));
    int sum = 0;
    sum += overflow_counter_FL * HIGH_LIMIT;
    sum += overflow_counter_FR * HIGH_LIMIT_FR;
    sum += cur_pan_pulses;
    sum += cur_tilt_pulses;
    int avg = sum / 2;

    // set overflows back to 0 so we can calculate distnace traveled next time
    overflow_counter_FL = 0;
    overflow_counter_FR = 0;

    // set counters in PCNT to 0 so we don't overcount in next call
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoders[PAN_ENCODER_ID].pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoders[TILT_ENCODER_ID].pcnt_unit));

    return avg;
}