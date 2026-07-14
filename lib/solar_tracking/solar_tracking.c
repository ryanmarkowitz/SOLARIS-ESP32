#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "solar_tracking.h"
#include <motor_driver.h>
#include <stdlib.h>
#include "freertos/semphr.h"
#include "shared_resources.h"
#include <encoders.h>

#include "solar_tracking.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SOLARIS_PT";

/*
X1 = 0, Y1 = 1
X2 = 2, Y2 = 3
*/

/*
FL = 4, FR = 5
RL = 6, RR = 7
*/

#define VOLTAGE_TOLERANCE 250

// ---------------------------------------------------------------------------
// Internal context
// ---------------------------------------------------------------------------

struct solaris_pt_ctx_t
{
    solaris_pt_config_t cfg;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool cali_enabled;
};

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

static void prv_gpio_output(int gpio)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, 0);
}

static void prv_mux_select(const struct solaris_pt_ctx_t *ctx, int channel)
{
    channel &= 0x07; /* CD4051B: valid channels 0..7 */

    /* channel = (C << 2) | (B << 1) | A */
    gpio_set_level(ctx->cfg.mux_sel_a, (channel >> 0) & 0x1);
    gpio_set_level(ctx->cfg.mux_sel_b, (channel >> 1) & 0x1);
    gpio_set_level(ctx->cfg.mux_sel_c, (channel >> 2) & 0x1);
    esp_rom_delay_us(ctx->cfg.mux_settle_us);
}

static int prv_adc_read_averaged(const struct solaris_pt_ctx_t *ctx)
{
    /* One throwaway read to let the SAR settle on the newly-switched input. */
    int discard = 0;
    (void)adc_oneshot_read(ctx->adc_handle, ctx->cfg.adc_channel, &discard);

    int32_t sum = 0;
    for (int i = 0; i < ctx->cfg.adc_samples; i++)
    {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(ctx->adc_handle, ctx->cfg.adc_channel, &raw);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(err));
            return -1;
        }
        sum += raw;
    }
    return (int)(sum / ctx->cfg.adc_samples);
}

static void prv_raw_to_result(const struct solaris_pt_ctx_t *ctx, int raw,
                              solaris_pt_result_t *out)
{
    out->raw = raw;
    /* Linear fallback across the attenuation's full-scale range. With
     * ADC_ATTEN_DB_12 the pin maxes near ~3100 mV, not the sensor VCC. */
    out->mv = (int)((raw / 4095.0f) * 3100.0f);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t solaris_pt_init(const solaris_pt_config_t *config, solaris_pt_handle_t *handle)
{
    if (!config || !handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->num_sensors < 1 || config->num_sensors > SOLARIS_PT_MAX_SENSORS)
    {
        ESP_LOGE(TAG, "num_sensors must be 1–%d", SOLARIS_PT_MAX_SENSORS);
        return ESP_ERR_INVALID_ARG;
    }

    struct solaris_pt_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }
    ctx->cfg = *config;

    // --- Select GPIOs ---
    prv_gpio_output(config->mux_sel_a);
    prv_gpio_output(config->mux_sel_b);
    prv_gpio_output(config->mux_sel_c);

    // --- ADC unit ---
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = config->adc_unit,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &ctx->adc_handle);
    if (err != ESP_OK)
    {
        /* ESP_ERR_INVALID_STATE here usually means this ADC unit is already
         * claimed elsewhere (e.g. by solaris_ultrasonic). Use a different unit. */
        ESP_LOGE(TAG, "ADC unit %d init failed: %s (already in use?)",
                 config->adc_unit, esp_err_to_name(err));
        free(ctx);
        return err;
    }

    // --- ADC channel (single common-output pin) ---
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = config->adc_bitwidth,
        .atten = config->adc_atten,
    };
    err = adc_oneshot_config_channel(ctx->adc_handle, config->adc_channel, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(ctx->adc_handle);
        free(ctx);
        return err;
    }

    // --- Calibration (curve fitting) ---
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = config->adc_unit,
        .chan = config->adc_channel,
        .atten = config->adc_atten,
        .bitwidth = config->adc_bitwidth,
    };
    ctx->cali_enabled =
        (adc_cali_create_scheme_curve_fitting(&cali_cfg, &ctx->cali_handle) == ESP_OK);
    if (!ctx->cali_enabled)
    {
        ESP_LOGW(TAG, "ADC calibration unavailable — using linear fallback");
    }

    ESP_LOGI(TAG, "Initialised: %d sensor(s), ADC unit %d ch %d, cali=%s",
             config->num_sensors, config->adc_unit, config->adc_channel,
             ctx->cali_enabled ? "yes" : "no");

    *handle = ctx;
    return ESP_OK;
}

esp_err_t solaris_pt_read_single(solaris_pt_handle_t handle, int index,
                                 solaris_pt_result_t *result)
{
    if (!handle || !result)
        return ESP_ERR_INVALID_ARG;
    if (index < 0 || index >= handle->cfg.num_sensors)
    {
        ESP_LOGE(TAG, "Index %d out of range (num_sensors=%d)",
                 index, handle->cfg.num_sensors);
        return ESP_ERR_INVALID_ARG;
    }

    prv_mux_select(handle, handle->cfg.channel_map[index]);
    int raw = prv_adc_read_averaged(handle);
    if (raw < 0)
        return ESP_FAIL;

    prv_raw_to_result(handle, raw, result);
    return ESP_OK;
}

esp_err_t solaris_pt_read(solaris_pt_handle_t handle, solaris_pt_result_t *results, bool is_tilt_panel_sensors)
{
    if (!handle || !results)
        return ESP_ERR_INVALID_ARG;

    // solar_tracking and driver_function both call this on the same mux-select
    // GPIOs + ADC channel -- without this, one task's mux_select can flip the
    // channel out from under the other task's pending ADC read.
    xSemaphoreTake(pt_bus_mutex, portMAX_DELAY);

    esp_err_t err = ESP_OK;
    for (int i = 0; i < SOLARIS_PT_MAX_SENSORS / 2; i++)
    {
        int index = is_tilt_panel_sensors ? i : i + 4;
        err = solaris_pt_read_single(handle, index, &results[i]);
        if (err != ESP_OK)
            break;
    }

    xSemaphoreGive(pt_bus_mutex);
    return err;
}

void solaris_pt_log(solaris_pt_handle_t handle, const solaris_pt_result_t *results)
{
    if (!handle || !results)
        return;

    for (int i = 0; i < handle->cfg.num_sensors; i++)
    {
        ESP_LOGI(TAG, "S%d (mux ch %d): %d mV  [raw=%d]",
                 i, handle->cfg.channel_map[i], results[i].mv, results[i].raw);
    }
}

esp_err_t solaris_pt_deinit(solaris_pt_handle_t handle)
{
    if (!handle)
        return ESP_ERR_INVALID_ARG;

    if (handle->cali_enabled && handle->cali_handle)
    {
        adc_cali_delete_scheme_curve_fitting(handle->cali_handle);
    }
    adc_oneshot_del_unit(handle->adc_handle);

    free(handle);
    ESP_LOGI(TAG, "Deinitialised");
    return ESP_OK;
}

// This function gets the information of the tilt pan phototrasnsistor sensors and drives the pan tilt motors accordingly
void solar_tracking(void *pvParameters)
{
    solaris_pt_result_t result[SOLARIS_PT_MAX_SENSORS / 2]; // buffer to store the read values from the phototransistors

    int left_right = 0,
        top_down = 0;

    int pulse_count;

    int tilt_pulse;
    int pan_pulse;

    bool panel_set = false;
    bool left_right_settled = false;
    bool up_down_settled = false;

    stop_motor(0);
    stop_motor(1);

    while (1)
    {
        xTaskNotifyStateClear(NULL);
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        panel_set = false;
        left_right_settled = false;
        up_down_settled = false;

        // Take the mutex once, blocking here until the drive motors give it
        // up. It's held for the entire adjustment pass below and only given
        // back once panel_set becomes true. A (non-recursive) mutex can't be
        // re-taken by the same task that already holds it, so this must
        // happen once here, not on every loop iteration below.
        xSemaphoreTake(actuator_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "Got The semaphore");

        while (!panel_set)
        {
            vTaskDelay(pdMS_TO_TICKS(250));

            solaris_pt_read(pt, result, true);
            // LEFT RIGHT CHECK
            left_right = (result[0].mv + result[2].mv) - (result[1].mv + result[3].mv);
            ESP_LOGI(TAG, "left_right value: %d", left_right);
            if (left_right > VOLTAGE_TOLERANCE)
            { // The panel needs to rotate left
                pulse_count = get_pulse_count(PANEL_TILT_ID);
                if (pulse_count + saved_tilt_position >= 0)
                {
                    // If panel can't move right anymore, mark left_right_settled boolean as true
                    if (panel_get_limit_state(PANEL_PAN_ID) == PANEL_AT_UPPER_LIMIT)
                    {
                        stop_motor(PANEL_PAN_ID);
                        left_right_settled = true;
                    }
                    else
                    {

                        motor_go_forward(PANEL_PAN_ID, .15);
                    }
                }
                else
                {
                    if (panel_get_limit_state(PANEL_PAN_ID) == PANEL_AT_LOWER_LIMIT)
                    {
                        stop_motor(PANEL_PAN_ID);
                        left_right_settled = true;
                    }
                    else
                    {

                        motor_go_backward(PANEL_PAN_ID, .15);
                    }
                }
            }
            else if (abs(left_right) > VOLTAGE_TOLERANCE)
            { // The panel needs to rotate right
                pulse_count = get_pulse_count(PANEL_TILT_ID);
                if (pulse_count + saved_tilt_position >= 0)
                {
                    if (panel_get_limit_state(PANEL_PAN_ID) == PANEL_AT_LOWER_LIMIT)
                    {
                        stop_motor(PANEL_PAN_ID);
                        left_right_settled = true;
                    }
                    else
                    {
                        motor_go_backward(PANEL_PAN_ID, .15);
                    }
                }
                else
                {
                    if (panel_get_limit_state(PANEL_PAN_ID) == PANEL_AT_UPPER_LIMIT)
                    {
                        stop_motor(PANEL_PAN_ID);
                        left_right_settled = true;
                    }
                    else
                    {
                        motor_go_forward(PANEL_PAN_ID, .15);
                    }
                }
            }
            else
            { // Found correct spot stop panning the motor
                left_right_settled = true;
                stop_motor(PANEL_PAN_ID);
            }

            // TOP DOWN CHECK
            top_down = (result[0].mv + result[1].mv) - (result[2].mv + result[3].mv);
            ESP_LOGI(TAG, "top_down value %d", top_down);
            if (top_down > VOLTAGE_TOLERANCE)
            { // The panel needs to pan up
                if (panel_get_limit_state(PANEL_TILT_ID) == PANEL_AT_UPPER_LIMIT)
                {
                    stop_motor(PANEL_TILT_ID);
                    up_down_settled = true;
                }
                else
                {
                    motor_go_forward(PANEL_TILT_ID, .15);
                }
            }
            else if (abs(top_down) > VOLTAGE_TOLERANCE)
            { // The panel needs to pan down
                if (panel_get_limit_state(PANEL_TILT_ID) == PANEL_AT_LOWER_LIMIT)
                {
                    stop_motor(PANEL_TILT_ID);
                    up_down_settled = true;
                }
                else
                {
                    motor_go_backward(PANEL_TILT_ID, .15);
                }
            }
            else
            { // Found correct spot stop panning the motor
                up_down_settled = true;
                stop_motor(PANEL_TILT_ID);
            }

            if (up_down_settled == true && left_right_settled == true)
            {
                panel_set = true;
            }

            // Only release the mutex once the panel has actually settled.
            if (panel_set)
            {
                xSemaphoreGive(actuator_mutex);
                ESP_LOGI(TAG, "Released the semaphore");
            }

            tilt_pulse = get_pulse_count(TILT_ENCODER_ID);
            pan_pulse = get_pulse_count(PAN_ENCODER_ID);
            ESP_LOGI(TAG, "pulse encoder val: %d", pan_pulse);
            ESP_LOGI(TAG, "tilt encoder val: %d", tilt_pulse);
        }
    }
}