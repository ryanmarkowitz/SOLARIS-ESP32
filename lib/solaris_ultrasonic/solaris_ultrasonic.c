/**
 * @file    solaris_ultrasonic.c
 * @brief   SOLARIS Ultrasonic Sensor Library — Implementation
 */

#include "solaris_ultrasonic.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SOLARIS_US";

// ---------------------------------------------------------------------------
// Internal context
// ---------------------------------------------------------------------------

struct solaris_us_ctx_t
{
    solaris_us_config_t cfg;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool cali_enabled;
    float mv_per_inch;
};

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

static void prv_mux_select(const struct solaris_us_ctx_t *ctx, int channel)
{
    gpio_set_level(ctx->cfg.mux_sel_a, (channel >> 0) & 0x1);
    gpio_set_level(ctx->cfg.mux_sel_b, (channel >> 1) & 0x1);
    esp_rom_delay_us(ctx->cfg.mux_settle_us);
}

static int prv_adc_read_averaged(const struct solaris_us_ctx_t *ctx)
{
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

static void prv_raw_to_result(const struct solaris_us_ctx_t *ctx, int raw, solaris_us_result_t *out)
{
    out->raw = raw;

    out->mv = (int)((raw / 4095.0f) * ctx->cfg.vcc_mv);
    out->mv = (out->mv / 2) + out->mv;

    out->inches = (float)out->mv / ctx->mv_per_inch;
    out->cm = out->inches * 2.54f;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t solaris_us_init(const solaris_us_config_t *config, solaris_us_handle_t *handle)
{
    if (!config || !handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->num_sensors < 1 || config->num_sensors > SOLARIS_US_MAX_SENSORS)
    {
        ESP_LOGE(TAG, "num_sensors must be 1–%d", SOLARIS_US_MAX_SENSORS);
        return ESP_ERR_INVALID_ARG;
    }

    struct solaris_us_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->cfg = *config;
    ctx->mv_per_inch = 5000.0f / 512.0f;

    // --- GPIO ---
    gpio_reset_pin(config->trigger_gpio);
    gpio_set_direction(config->trigger_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->trigger_gpio, 0);

    gpio_reset_pin(config->mux_sel_a);
    gpio_set_direction(config->mux_sel_a, GPIO_MODE_OUTPUT);
    gpio_set_level(config->mux_sel_a, 0);

    gpio_reset_pin(config->mux_sel_b);
    gpio_set_direction(config->mux_sel_b, GPIO_MODE_OUTPUT);
    gpio_set_level(config->mux_sel_b, 0);

    // --- ADC unit ---
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = config->adc_unit,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &ctx->adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        free(ctx);
        return err;
    }

    // --- ADC channel ---
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

    // --- Calibration (curve-fitting, best available) ---
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = config->adc_unit,
        .atten = config->adc_atten,
        .bitwidth = config->adc_bitwidth,
        .chan = config->adc_channel,
    };
    ctx->cali_enabled =
        (adc_cali_create_scheme_curve_fitting(&cali_cfg, &ctx->cali_handle) == ESP_OK);

    if (!ctx->cali_enabled)
    {
        ESP_LOGW(TAG, "ADC calibration unavailable — using linear fallback");
    }

    ESP_LOGI(TAG, "Initialised: %d sensor(s), VCC=%dmV, %.2f mV/inch, cali=%s",
             config->num_sensors, config->vcc_mv, ctx->mv_per_inch,
             ctx->cali_enabled ? "yes" : "no");

    *handle = ctx;
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_us_trigger(solaris_us_handle_t handle)
{
    if (!handle)
        return ESP_ERR_INVALID_ARG;

    gpio_set_level(handle->cfg.trigger_gpio, 1);
    esp_rom_delay_us(handle->cfg.trigger_pulse_us);
    gpio_set_level(handle->cfg.trigger_gpio, 0);
    esp_rom_delay_us(handle->cfg.trigger_settle_us);

    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_us_read_single(solaris_us_handle_t handle, int index, solaris_us_result_t *result)
{
    if (!handle || !result)
        return ESP_ERR_INVALID_ARG;
    if (index < 0 || index >= handle->cfg.num_sensors)
    {
        ESP_LOGE(TAG, "Index %d out of range (num_sensors=%d)",
                 index, handle->cfg.num_sensors);
        return ESP_ERR_INVALID_ARG;
    }

    prv_mux_select(handle, index);
    int raw = prv_adc_read_averaged(handle);
    if (raw < 0)
        return ESP_FAIL;

    prv_raw_to_result(handle, raw, result);
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_us_read(solaris_us_handle_t handle, solaris_us_result_t *results)
{
    if (!handle || !results)
        return ESP_ERR_INVALID_ARG;

    // Fire trigger — all sensors range simultaneously
    esp_err_t err = solaris_us_trigger(handle);
    if (err != ESP_OK)
        return err;

    // Wait for ranging cycle to complete
    vTaskDelay(pdMS_TO_TICKS(handle->cfg.ranging_delay_ms));

    // Read each mux channel
    for (int i = 0; i < handle->cfg.num_sensors; i++)
    {
        err = solaris_us_read_single(handle, i, &results[i]);
        if (err != ESP_OK)
            return err;
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------

void solaris_us_log(solaris_us_handle_t handle, const solaris_us_result_t *results)
{
    if (!handle || !results)
        return;

    for (int i = 0; i < handle->cfg.num_sensors; i++)
    {
        ESP_LOGI(TAG, "S%d: %5.2f in (%4.1f cm)  [raw=%d, %d mV]",
                 i, results[i].inches, results[i].cm,
                 results[i].raw, results[i].mv);
    }
}

// ---------------------------------------------------------------------------

void solaris_us_print_teleplot(solaris_us_handle_t handle, const solaris_us_result_t *results)
{
    if (!handle || !results)
        return;

    for (int i = 0; i < handle->cfg.num_sensors; i++)
    {
        printf(">S%d:%.2f ", i, results[i].inches);
    }
    printf("\n");
}

// ---------------------------------------------------------------------------

esp_err_t solaris_us_deinit(solaris_us_handle_t handle)
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