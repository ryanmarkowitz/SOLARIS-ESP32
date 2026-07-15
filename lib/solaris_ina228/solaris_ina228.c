/**
 * @file    solaris_ina228.c
 * @brief   SOLARIS INA228 Power Monitor Library — Implementation
 */

#include "solaris_ina228.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <quickselect.h>
#include <solaris_weather.h>
#include <time.h>
#include "shared_resources.h"
#include <motor_driver.h>

static const char *TAG = "SOLARIS_INA228";

// INA228 constants
#define INA228_MANUFACTURER_ID_VAL 0x5449
#define INA228_VBUS_LSB_V 0.0001953125f // 195.3125 uV per LSB
#define INA228_TEMP_LSB_C 0.0078125f    // 7.8125 m°C per LSB
#define INA228_POWER_COEFF 3.2f
#define INA228_ENERGY_LSB_COEFF 16.0f // Energy LSB = 16 * 3.2 * CURRENT_LSB
#define INA228_CHARGE_LSB_COEFF 1.0f  // Charge LSB = CURRENT_LSB
#define COULOMBS_PER_MAH 3.6f

// RSTACC bit in CONFIG register — resets energy and charge accumulators
#define INA228_CONFIG_RSTACC (1 << 14)

int solaris_power_buffer_idx = 0;
int solaris_power_buffer_with_moves_included_idx = 0;
float solaris_power_buffer[SOLARIS_RING_BUFFER_SIZE];
float solaris_power_buffer_with_moves_included[60];

// ---------------------------------------------------------------------------
// Internal context
// ---------------------------------------------------------------------------

struct solaris_ina228_ctx_t
{
    solaris_ina228_config_t cfg;
    float initial_soc;
    bool soc_initialized;
};

// ---------------------------------------------------------------------------
// Private helpers — raw register I/O
// ---------------------------------------------------------------------------

static esp_err_t prv_write_reg16(const struct solaris_ina228_ctx_t *ctx,
                                 uint8_t reg, uint16_t data)
{
    uint8_t buf[3] = {reg, (data >> 8) & 0xFF, data & 0xFF};
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_to_device(ctx->cfg.i2c_port, ctx->cfg.i2c_addr,
                                               buf, 3,
                                               pdMS_TO_TICKS(ctx->cfg.timeout_ms));
    xSemaphoreGive(i2c_bus_mutex);
    return err;
}

static esp_err_t prv_read_reg16(const struct solaris_ina228_ctx_t *ctx,
                                uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_read_device(ctx->cfg.i2c_port,
                                                 ctx->cfg.i2c_addr,
                                                 &reg, 1, buf, 2,
                                                 pdMS_TO_TICKS(ctx->cfg.timeout_ms));
    xSemaphoreGive(i2c_bus_mutex);
    if (err == ESP_OK)
    {
        *out = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return err;
}

static esp_err_t prv_read_reg24(const struct solaris_ina228_ctx_t *ctx,
                                uint8_t reg, uint32_t *out)
{
    uint8_t buf[3];
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_read_device(ctx->cfg.i2c_port,
                                                 ctx->cfg.i2c_addr,
                                                 &reg, 1, buf, 3,
                                                 pdMS_TO_TICKS(ctx->cfg.timeout_ms));
    xSemaphoreGive(i2c_bus_mutex);
    if (err == ESP_OK)
    {
        *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }
    return err;
}

static esp_err_t prv_read_reg40(const struct solaris_ina228_ctx_t *ctx,
                                uint8_t reg, uint64_t *out)
{
    uint8_t buf[5];
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_read_device(ctx->cfg.i2c_port,
                                                 ctx->cfg.i2c_addr,
                                                 &reg, 1, buf, 5,
                                                 pdMS_TO_TICKS(ctx->cfg.timeout_ms));
    xSemaphoreGive(i2c_bus_mutex);
    if (err == ESP_OK)
    {
        *out = ((uint64_t)buf[0] << 32) |
               ((uint64_t)buf[1] << 24) |
               ((uint64_t)buf[2] << 16) |
               ((uint64_t)buf[3] << 8) |
               (uint64_t)buf[4];
    }
    return err;
}

// ---------------------------------------------------------------------------
// Private helpers — conversion
// ---------------------------------------------------------------------------

static int32_t prv_sign_extend_20bit(uint32_t val)
{
    if (val & 0x80000)
        return (int32_t)(val | 0xFFF00000);
    return (int32_t)val;
}

static float prv_calculate_soc(const struct solaris_ina228_ctx_t *ctx,
                               float voltage_v, float charge_c)
{
    // Voltage anchor — full charge resets SOC to 100%
    if (voltage_v >= ctx->cfg.battery_full_v)
    {
        return 100.0f;
    }
    // Voltage anchor — empty resets SOC to 0%
    if (voltage_v <= ctx->cfg.battery_empty_v)
    {
        return 0.0f;
    }

    // Coulomb counting
    float capacity_c = ctx->cfg.battery_capacity_mah * COULOMBS_PER_MAH;
    float used_c = charge_c; // CHARGE register counts net coulombs
    float soc = ctx->initial_soc - (used_c / capacity_c) * 100.0f;

    // Clamp
    if (soc > 100.0f)
        soc = 100.0f;
    if (soc < 0.0f)
        soc = 0.0f;

    return soc;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_init(const solaris_ina228_config_t *config,
                              solaris_ina228_handle_t *handle)
{
    if (!config || !handle)
        return ESP_ERR_INVALID_ARG;

    struct solaris_ina228_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
        return ESP_ERR_NO_MEM;

    ctx->cfg = *config;
    ctx->initial_soc = 100.0f;
    ctx->soc_initialized = false;
    esp_err_t err;

    // --- Verify device ---
    uint16_t mfg_id = 0;
    err = prv_read_reg16(ctx, INA228_REG_MANUFACTURER_ID, &mfg_id);
    if (err != ESP_OK || mfg_id != INA228_MANUFACTURER_ID_VAL)
    {
        ESP_LOGE(TAG, "INA228 not found! Manufacturer ID: 0x%04X", mfg_id);
        free(ctx);
        return ESP_ERR_NOT_FOUND;
    }

    // --- Write configuration registers ---
    ESP_ERROR_CHECK(prv_write_reg16(ctx, INA228_REG_CONFIG, config->config_reg));
    ESP_ERROR_CHECK(prv_write_reg16(ctx, INA228_REG_ADC_CONFIG, config->adc_config_reg));
    ESP_ERROR_CHECK(prv_write_reg16(ctx, INA228_REG_SHUNT_CAL, config->shunt_cal));

    ESP_LOGI(TAG, "INA228 initialised. CURRENT_LSB=%.2e A, SHUNT_CAL=%d, addr=0x%02X",
             config->current_lsb, config->shunt_cal, config->i2c_addr);

    *handle = ctx;
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_read_voltage(solaris_ina228_handle_t handle,
                                      float *voltage_v)
{
    if (!handle || !voltage_v)
        return ESP_ERR_INVALID_ARG;

    uint32_t raw = 0;
    esp_err_t err = prv_read_reg24(handle, INA228_REG_VBUS, &raw);
    if (err != ESP_OK)
        return err;

    *voltage_v = (float)(raw >> 4) * INA228_VBUS_LSB_V;
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_read_current(solaris_ina228_handle_t handle,
                                      float *current_a)
{
    if (!handle || !current_a)
        return ESP_ERR_INVALID_ARG;

    uint32_t raw = 0;
    esp_err_t err = prv_read_reg24(handle, INA228_REG_CURRENT, &raw);
    if (err != ESP_OK)
        return err;

    int32_t signed_counts = prv_sign_extend_20bit(raw >> 4);
    *current_a = (float)signed_counts * handle->cfg.current_lsb;
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_read_soc(solaris_ina228_handle_t handle,
                                  float *soc_percent)
{
    if (!handle || !soc_percent)
        return ESP_ERR_INVALID_ARG;

    float voltage_v = 0.0f;
    esp_err_t err = solaris_ina228_read_voltage(handle, &voltage_v);
    if (err != ESP_OK)
        return err;

    // Read CHARGE register (40-bit)
    uint64_t raw_charge = 0;
    err = prv_read_reg40(handle, INA228_REG_CHARGE, &raw_charge);
    if (err != ESP_OK)
        return err;

    // Charge LSB = CURRENT_LSB coulombs
    float charge_c = (float)(int64_t)raw_charge * handle->cfg.current_lsb;

    // On first SOC read, anchor initial SOC to voltage
    if (!handle->soc_initialized)
    {
        float range = handle->cfg.battery_full_v - handle->cfg.battery_empty_v;
        float clamped = voltage_v;
        if (clamped > handle->cfg.battery_full_v)
            clamped = handle->cfg.battery_full_v;
        if (clamped < handle->cfg.battery_empty_v)
            clamped = handle->cfg.battery_empty_v;
        handle->initial_soc = ((clamped - handle->cfg.battery_empty_v) / range) * 100.0f;
        handle->soc_initialized = true;
        ESP_LOGI(TAG, "SOC initialised from voltage: %.1f%%", handle->initial_soc);
    }

    *soc_percent = prv_calculate_soc(handle, voltage_v, charge_c);
    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_read(solaris_ina228_handle_t handle,
                              solaris_ina228_result_t *result)
{
    if (!handle || !result)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    // VBUS
    uint32_t raw_vbus = 0;
    err = prv_read_reg24(handle, INA228_REG_VBUS, &raw_vbus);
    if (err != ESP_OK)
        return err;
    result->voltage_v = (float)(raw_vbus >> 4) * INA228_VBUS_LSB_V;

    // CURRENT
    uint32_t raw_curr = 0;
    err = prv_read_reg24(handle, INA228_REG_CURRENT, &raw_curr);
    if (err != ESP_OK)
        return err;
    int32_t signed_counts = prv_sign_extend_20bit(raw_curr >> 4);
    result->current_a = (float)signed_counts * handle->cfg.current_lsb;
    result->current_ma = result->current_a * 1000.0f;

    // POWER
    uint32_t raw_pwr = 0;
    err = prv_read_reg24(handle, INA228_REG_POWER, &raw_pwr);
    if (err != ESP_OK)
        return err;
    result->power_w = (float)raw_pwr * handle->cfg.current_lsb * INA228_POWER_COEFF;

    // ENERGY (40-bit)
    uint64_t raw_energy = 0;
    err = prv_read_reg40(handle, INA228_REG_ENERGY, &raw_energy);
    if (err != ESP_OK)
        return err;
    result->energy_j = (float)raw_energy * INA228_ENERGY_LSB_COEFF *
                       INA228_POWER_COEFF * handle->cfg.current_lsb;

    // CHARGE (40-bit)
    uint64_t raw_charge = 0;
    err = prv_read_reg40(handle, INA228_REG_CHARGE, &raw_charge);
    if (err != ESP_OK)
        return err;
    result->charge_c = (float)(int64_t)raw_charge * handle->cfg.current_lsb;
    result->charge_mah = result->charge_c / COULOMBS_PER_MAH;

    // TEMPERATURE
    uint16_t raw_temp = 0;
    err = prv_read_reg16(handle, INA228_REG_DIETEMP, &raw_temp);
    if (err != ESP_OK)
        return err;
    result->temperature_c = (float)(int16_t)raw_temp * INA228_TEMP_LSB_C;

    // SOC
    if (!handle->soc_initialized)
    {
        float range = handle->cfg.battery_full_v - handle->cfg.battery_empty_v;
        float clamped = result->voltage_v;
        if (clamped > handle->cfg.battery_full_v)
            clamped = handle->cfg.battery_full_v;
        if (clamped < handle->cfg.battery_empty_v)
            clamped = handle->cfg.battery_empty_v;
        handle->initial_soc = ((clamped - handle->cfg.battery_empty_v) / range) * 100.0f;
        handle->soc_initialized = true;
        ESP_LOGI(TAG, "SOC initialised from voltage: %.1f%%", handle->initial_soc);
    }
    result->soc_percent = prv_calculate_soc(handle, result->voltage_v, result->charge_c);

    return ESP_OK;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_reset_accumulators(solaris_ina228_handle_t handle)
{
    if (!handle)
        return ESP_ERR_INVALID_ARG;

    uint16_t val = handle->cfg.config_reg | INA228_CONFIG_RSTACC;
    esp_err_t err = prv_write_reg16(handle, INA228_REG_CONFIG, val);
    if (err != ESP_OK)
        return err;

    // Restore normal config (RSTACC is self-clearing but write it back cleanly)
    err = prv_write_reg16(handle, INA228_REG_CONFIG, handle->cfg.config_reg);

    handle->soc_initialized = false;
    ESP_LOGI(TAG, "Accumulators reset");
    return err;
}

// ---------------------------------------------------------------------------

void solaris_ina228_log(solaris_ina228_handle_t handle,
                        const solaris_ina228_result_t *result)
{
    if (!handle || !result)
        return;

    ESP_LOGI(TAG, "V: %6.3fV | I: %8.4fA (%7.2fmA) | P: %6.4fW | "
                  "SOC: %5.1f%% | Temp: %.1f°C | Charge: %.2fmAh | Energy: %.4fJ",
             result->voltage_v,
             result->current_a, result->current_ma,
             result->power_w,
             result->soc_percent,
             result->temperature_c,
             result->charge_mah,
             result->energy_j);
}

// ---------------------------------------------------------------------------

void solaris_ina228_print_teleplot(solaris_ina228_handle_t handle,
                                   const solaris_ina228_result_t *result)
{
    if (!handle || !result)
        return;

    printf(">V:%.3f >I:%.4f >P:%.4f >SOC:%.1f >T:%.1f\n",
           result->voltage_v,
           result->current_a,
           result->power_w,
           result->soc_percent,
           result->temperature_c);
}

// ---------------------------------------------------------------------------

esp_err_t solaris_ina228_deinit(solaris_ina228_handle_t handle)
{
    if (!handle)
        return ESP_ERR_INVALID_ARG;

    i2c_driver_delete(handle->cfg.i2c_port);
    free(handle);
    ESP_LOGI(TAG, "Deinitialised");
    return ESP_OK;
}

void solaris_ina228_make_move_decision(void *pvParameters)
{
    solaris_ina228_handle_t handle = (solaris_ina228_handle_t)pvParameters;
    float median_last_30s;
    float median_last_3m;
    solaris_weather_t weather;
    solaris_ina228_result_t result;
    uint32_t now;
    uint32_t bits;
    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        solaris_weather_get(&weather);
        now = (uint32_t)time(NULL);
        // If its an hour after sunruse or before, don't bother moving
        if (now - 3600 < weather.sunrise)
        {
            ;
        }
        // if its an hour before sunset or after, dont bother moving
        else if (now + 3600 > weather.sunset)
        {
            ;
        }
        else
        {
            xSemaphoreTake(solaris_energy_monitor_resource, portMAX_DELAY);
            median_last_30s = solaris_windowed_median(solaris_power_buffer, SOLARIS_RING_BUFFER_SIZE, solaris_power_buffer_idx, 30);
            median_last_3m = solaris_windowed_median(solaris_power_buffer, SOLARIS_RING_BUFFER_SIZE, solaris_power_buffer_idx, SOLARIS_RING_BUFFER_SIZE);
            xSemaphoreGive(solaris_energy_monitor_resource);
            // If the last 30 seconds has seen a 5% drop it is likely shade / clouds over the panel.
            // Determine if cloud percentage is low and then move the robot if that's the case
            solaris_ina228_read(handle, &result);
            if (result.current_a != 0)
            { // If current is at 0.0mA then the battery is likely fully charged. Don't bother moving

                if (median_last_30s * 1.05 < median_last_3m)
                {

                    const solaris_forecast_entry_t *forecast_now = solaris_weather_get_forecast_at(&weather, now);
                    uint8_t cloud_cover_pct = forecast_now ? forecast_now->cloud_cover_pct : 0;
                    // If cloud coverage percentage is high either don't move at all, or add extra logic on if you should move
                    if (cloud_cover_pct > CLOUD_COVERAGE_PERCENTAGE_DECISION)
                    {
                        ;
                    }
                    else
                    {
                        // Send notification to move
                        memcpy(&bits, &median_last_3m, sizeof(bits));
                        xTaskNotify(xDriverFunction, bits, eSetValueWithOverwrite);
                    }
                }
            }
        }
    }
}

void solaris_ina228_1s_read(void *pvParmaters)
{
    solaris_ina228_handle_t handle = (solaris_ina228_handle_t)pvParmaters;
    solaris_ina228_result_t result;
    int counter = 0;
    TickType_t last = xTaskGetTickCount();
    while (1)
    {
        // don't keep track unless the robot isn't moving currently Also grab the mutex for writing to the shared buffer
        if (xSemaphoreTake(actuator_mutex, 0) == pdTRUE)
        {
            xSemaphoreTake(solaris_energy_monitor_resource, portMAX_DELAY);
            xSemaphoreTake(solaris_energy_monitor_resource_with_moves, portMAX_DELAY);
            // Read the energy units power and store the result in the ring buffer
            solaris_ina228_read(handle, &result);

            solaris_power_buffer[solaris_power_buffer_idx] = -result.power_w;
            solaris_power_buffer_idx = (solaris_power_buffer_idx + 1) % 180;
            solaris_power_buffer_with_moves_included[solaris_power_buffer_with_moves_included_idx] = -result.power_w;
            solaris_power_buffer_with_moves_included_idx = (solaris_power_buffer_with_moves_included_idx + 1) % 180;

            // give back mutexes
            xSemaphoreGive(solaris_energy_monitor_resource);
            xSemaphoreGive(actuator_mutex);
            xSemaphoreGive(solaris_energy_monitor_resource_with_moves);

            counter++;
            if (counter == SOLARIS_RING_BUFFER_SIZE)
            {
                // after 3 minutes of information is filled, notify move decision task to fire
                counter = 0;
                xTaskNotifyGive(xMoveDecision);
            }
        }
        else // Solaris is moving. In the event it is moving only add value to the with moves buffer
        {
            xSemaphoreTake(solaris_energy_monitor_resource_with_moves, portMAX_DELAY);

            solaris_ina228_read(handle, &result);
            solaris_power_buffer_with_moves_included[solaris_power_buffer_with_moves_included_idx] = result.power_w;
            solaris_power_buffer_with_moves_included_idx = (solaris_power_buffer_with_moves_included_idx + 1) % 180;

            xSemaphoreGive(solaris_energy_monitor_resource_with_moves);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}