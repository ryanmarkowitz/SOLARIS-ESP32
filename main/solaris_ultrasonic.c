/**
 * @file    solaris_ultrasonic.c
 * @brief   SOLARIS Ultrasonic Sensor Library — Implementation
 *
 * Only the triggering was changed vs. the previous version: reads now hold RX
 * high across several ranging cycles so the AN line settles before sampling.
 * The voltage/distance math is unchanged.
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

struct solaris_us_ctx_t {
    solaris_us_config_t       cfg;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t         cali_handle;
    bool                      cali_enabled;
    float                     mv_per_inch;
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
    for (int i = 0; i < ctx->cfg.adc_samples; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(ctx->adc_handle, ctx->cfg.adc_channel, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(err));
            return -1;
        }
        sum += raw;
    }
    return (int)(sum / ctx->cfg.adc_samples);
}

/* Voltage/distance conversion — UNCHANGED from the working design.
 * pin voltage -> x1.5 (undo the 1k/2k divider) -> inches via vcc_mv/512. */
static void prv_raw_to_result(const struct solaris_us_ctx_t *ctx, int raw, solaris_us_result_t *out)
{
    out->raw = raw;

    if (ctx->cali_enabled) {
        adc_cali_raw_to_voltage(ctx->cali_handle, raw, &out->mv);
    } else {
        out->mv = (int)((raw / 4095.0f) * ctx->cfg.vcc_mv);
    }
    out->mv = (out->mv / 2) + out->mv;   /* x1.5: recover full 0..5V AN swing */

    out->inches = (float)out->mv / ctx->mv_per_inch;
    out->cm     = out->inches * 2.54f;
}

/* Command ranging by holding RX HIGH across several ~49 ms cycles so the AN
 * line ramps to the true distance voltage before we sample it. */
static void prv_range_burst(const struct solaris_us_ctx_t *ctx)
{
    gpio_set_level(ctx->cfg.trigger_gpio, 1);

    int cycle_ms = (ctx->cfg.ranging_cycle_ms > 0) ? ctx->cfg.ranging_cycle_ms : 49;
    int cycles   = (ctx->cfg.settle_cycles   > 0) ? ctx->cfg.settle_cycles   : 1;
    vTaskDelay(pdMS_TO_TICKS(cycle_ms * cycles));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t solaris_us_init(const solaris_us_config_t *config, solaris_us_handle_t *handle)
{
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->num_sensors < 1 || config->num_sensors > SOLARIS_US_MAX_SENSORS) {
        ESP_LOGE(TAG, "num_sensors must be 1–%d", SOLARIS_US_MAX_SENSORS);
        return ESP_ERR_INVALID_ARG;
    }

    struct solaris_us_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    ctx->cfg = *config;
    ctx->mv_per_inch = (float)config->vcc_mv / 512.0f;

    // --- GPIO ---
    gpio_reset_pin(config->trigger_gpio);
    gpio_set_direction(config->trigger_gpio, GPIO_MODE_OUTPUT);
    /* free_run: hold RX high so sensors range continuously.
     * triggered: idle low; each read raises RX for a burst. */
    gpio_set_level(config->trigger_gpio, config->free_run ? 1 : 0);

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
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        free(ctx);
        return err;
    }

    // --- ADC channel ---
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = config->adc_bitwidth,
        .atten    = config->adc_atten,
    };
    err = adc_oneshot_config_channel(ctx->adc_handle, config->adc_channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(ctx->adc_handle);
        free(ctx);
        return err;
    }

    // --- Calibration (curve-fitting) ---
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = config->adc_unit,
        .atten    = config->adc_atten,
        .bitwidth = config->adc_bitwidth,
        .chan     = config->adc_channel,
    };
    ctx->cali_enabled =
        (adc_cali_create_scheme_curve_fitting(&cali_cfg, &ctx->cali_handle) == ESP_OK);
    if (!ctx->cali_enabled) {
        ESP_LOGW(TAG, "ADC calibration unavailable — using linear fallback");
    }

    /* MaxSonar needs ~250 ms after power before it accepts RX, and calibrates
     * its ringdown on the first cycle — keep the area clear during startup. */
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "Initialised: %d sensor(s), VCC=%dmV, %.2f mV/inch, %s, cali=%s",
             config->num_sensors, config->vcc_mv, ctx->mv_per_inch,
             config->free_run ? "free-run" : "triggered",
             ctx->cali_enabled ? "yes" : "no");

    *handle = ctx;
    return ESP_OK;
}

esp_err_t solaris_us_trigger(solaris_us_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (handle->cfg.free_run) return ESP_OK;   /* RX already tied high */

    prv_range_burst(handle);
    gpio_set_level(handle->cfg.trigger_gpio, 0);  /* stop; AN holds last value */
    return ESP_OK;
}

esp_err_t solaris_us_read_single(solaris_us_handle_t handle, int index, solaris_us_result_t *result)
{
    if (!handle || !result) return ESP_ERR_INVALID_ARG;
    if (index < 0 || index >= handle->cfg.num_sensors) {
        ESP_LOGE(TAG, "Index %d out of range (num_sensors=%d)",
                 index, handle->cfg.num_sensors);
        return ESP_ERR_INVALID_ARG;
    }

    prv_mux_select(handle, index);
    int raw = prv_adc_read_averaged(handle);
    if (raw < 0) return ESP_FAIL;

    prv_raw_to_result(handle, raw, result);
    return ESP_OK;
}

esp_err_t solaris_us_read(solaris_us_handle_t handle, solaris_us_result_t *results)
{
    if (!handle || !results) return ESP_ERR_INVALID_ARG;

    /* Triggered mode: hold RX high across several cycles so every sensor's AN
     * line settles, THEN read each channel while ranging is still active. */
    if (!handle->cfg.free_run) {
        prv_range_burst(handle);
    }

    esp_err_t err = ESP_OK;
    for (int i = 0; i < handle->cfg.num_sensors; i++) {
        err = solaris_us_read_single(handle, i, &results[i]);
        if (err != ESP_OK) break;
    }

    if (!handle->cfg.free_run) {
        gpio_set_level(handle->cfg.trigger_gpio, 0);  /* stop ranging */
    }
    return err;
}

void solaris_us_log(solaris_us_handle_t handle, const solaris_us_result_t *results)
{
    if (!handle || !results) return;

    for (int i = 0; i < handle->cfg.num_sensors; i++) {
        ESP_LOGI(TAG, "S%d: %5.2f in (%4.1f cm)  [raw=%d, %d mV]",
                 i, results[i].inches, results[i].cm,
                 results[i].raw, results[i].mv);
    }
}

void solaris_us_print_teleplot(solaris_us_handle_t handle, const solaris_us_result_t *results)
{
    if (!handle || !results) return;

    for (int i = 0; i < handle->cfg.num_sensors; i++) {
        printf(">S%d:%.2f ", i, results[i].inches);
    }
    printf("\n");
}

esp_err_t solaris_us_deinit(solaris_us_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    if (handle->cali_enabled && handle->cali_handle) {
        adc_cali_delete_scheme_curve_fitting(handle->cali_handle);
    }
    adc_oneshot_del_unit(handle->adc_handle);

    free(handle);
    ESP_LOGI(TAG, "Deinitialised");
    return ESP_OK;
}