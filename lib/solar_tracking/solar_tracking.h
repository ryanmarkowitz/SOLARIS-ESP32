/**
 * @file    solaris_phototransistor.h
 * @brief   SOLARIS Phototransistor Sensor Library
 *
 * Supports up to 8 analog phototransistor sensors multiplexed through a
 * CD4051B 8-channel analog mux, sampled via a single ESP32 ADC pin
 * (oneshot driver). Intended for the solar-tracking light sensors
 * (e.g. X1/X2/Y1/Y2).
 *
 * Wiring summary:
 *   - Each phototransistor output → a CD4051B channel input (CH0..CH7)
 *   - CD4051B COMMON OUT/IN (pin 3) → ESP32 ADC pin
 *   - CD4051B A (pin 11) → SOLARIS_PT config .mux_sel_a  (select bit 0)
 *   - CD4051B B (pin 10) → SOLARIS_PT config .mux_sel_b  (select bit 1)
 *   - CD4051B C (pin 9)  → SOLARIS_PT config .mux_sel_c  (select bit 2)
 *
 * CD4051B channel select (INH = 0), per datasheet Section 7.4:
 *      channel = (C << 2) | (B << 1) | A          (A = LSB, C = MSB)
 *
 * -------------------------------------------------------------------------
 * COEXISTENCE with solaris_ultrasonic:
 *   This library defaults to ADC_UNIT_2 so it can run alongside
 *   solaris_ultrasonic (which uses ADC_UNIT_1) without contention. A given
 *   physical ADC unit can only be claimed by ONE adc_oneshot unit handle,
 *   so keep the two libraries on different units. If you move this library
 *   to ADC_UNIT_1, it will collide with solaris_ultrasonic's ADC unit.
 *
 *   NOTE: ADC2 on the ESP32-S3 is shared with the Wi-Fi radio. If Wi-Fi is
 *   active, ADC2 reads can fail. For a standalone tracker this is fine.
 * -------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/** Maximum number of sensors the library supports (CD4051B = 8 channels). */
#define SOLARIS_PT_MAX_SENSORS 8

    /** Convenience indices for the solar-tracker sensor layout. These are just
     *  indices into the results array / channel_map. Rename to match your rig
     *  (e.g. a quadrant/octant layout for the tracker); these are just aliases
     *  for array indices 0..7. */
    enum
    {
        SOLARIS_PT_S0 = 0,
        SOLARIS_PT_S1 = 1,
        SOLARIS_PT_S2 = 2,
        SOLARIS_PT_S3 = 3,
        SOLARIS_PT_S4 = 4,
        SOLARIS_PT_S5 = 5,
        SOLARIS_PT_S6 = 6,
        SOLARIS_PT_S7 = 7,
    };

    // ---------------------------------------------------------------------------
    // Configuration struct
    // ---------------------------------------------------------------------------

    /**
     * @brief  Configuration passed to solaris_pt_init().
     *
     * Populate via SOLARIS_PT_CONFIG_DEFAULT() then override individual fields.
     */
    typedef struct
    {
        /* MUX select lines (CD4051B has three address lines). */
        int mux_sel_a; /**< CD4051B A (pin 11) — channel select bit 0. */
        int mux_sel_b; /**< CD4051B B (pin 10) — channel select bit 1. */
        int mux_sel_c; /**< CD4051B C (pin 9)  — channel select bit 2. */

        /* ADC (wired to the mux COMMON OUT/IN pin). */
        adc_unit_t adc_unit;         /**< ADC unit. Keep != solaris_ultrasonic's. */
        adc_channel_t adc_channel;   /**< ADC channel connected to mux output. */
        adc_atten_t adc_atten;       /**< Attenuation (ADC_ATTEN_DB_12 ≈ 0–3.1 V). */
        adc_bitwidth_t adc_bitwidth; /**< Bit width (ADC_BITWIDTH_12 on the S3). */
        int adc_samples;             /**< Averaged samples per read (e.g. 16). */

        /* Logical sensor -> mux channel mapping. */
        int num_sensors;                         /**< 1..SOLARIS_PT_MAX_SENSORS. */
        int channel_map[SOLARIS_PT_MAX_SENSORS]; /**< channel_map[i] = mux ch for sensor i. */

        /* Timing. */
        int mux_settle_us; /**< Settle time after switching channels, in µs. */
    } solaris_pt_config_t;

/**
 * @brief  Sensible defaults: 8 sensors on mux channels 0–7, ADC2 (GPIO11).
 *
 * Example:
 * @code
 *   solaris_pt_config_t cfg = SOLARIS_PT_CONFIG_DEFAULT();
 *   cfg.mux_sel_a = 15;   // adjust to your wiring
 *   solaris_pt_handle_t pt;
 *   solaris_pt_init(&cfg, &pt);
 * @endcode
 */
#define SOLARIS_PT_CONFIG_DEFAULT() {          \
    .mux_sel_a = 10,                           \
    .mux_sel_b = 12,                           \
    .mux_sel_c = 13,                           \
    .adc_unit = ADC_UNIT_2,                    \
    .adc_channel = ADC_CHANNEL_0, /* GPIO11 */ \
    .adc_atten = ADC_ATTEN_DB_12,              \
    .adc_bitwidth = ADC_BITWIDTH_12,           \
    .adc_samples = 16,                         \
    .num_sensors = 8,                          \
    .channel_map = {0, 1, 2, 3, 4, 5, 6, 7},   \
    .mux_settle_us = 150,                      \
}

    // ---------------------------------------------------------------------------
    // Handle
    // ---------------------------------------------------------------------------

    /** Opaque handle returned by solaris_pt_init(). */
    typedef struct solaris_pt_ctx_t *solaris_pt_handle_t;

    // ---------------------------------------------------------------------------
    // Measurement result
    // ---------------------------------------------------------------------------

    /**
     * @brief  Per-sensor measurement result.
     */
    typedef struct
    {
        int raw; /**< Averaged raw ADC reading (0..4095). */
        int mv;  /**< Voltage in millivolts (calibrated if available). */
    } solaris_pt_result_t;

    // ---------------------------------------------------------------------------
    // API
    // ---------------------------------------------------------------------------

    /**
     * @brief  Initialise the SOLARIS phototransistor subsystem.
     *
     * Configures the mux select GPIOs, the ADC unit/channel, and optional
     * curve-fitting calibration. Call once before any other solaris_pt_* call.
     *
     * @param[in]  config   Pointer to a populated solaris_pt_config_t.
     * @param[out] handle   Receives the allocated context handle on success.
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t solaris_pt_init(const solaris_pt_config_t *config,
                              solaris_pt_handle_t *handle);

    /**
     * @brief  Read every configured sensor into @p results.
     *
     * Cycles through each mux channel, settling and averaging per channel.
     * (Phototransistors are continuous — no trigger pulse is needed.)
     *
     * @param[in]  handle   Handle from solaris_pt_init().
     * @param[out] results  Array of at least config->num_sensors elements.
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t solaris_pt_read(solaris_pt_handle_t handle,
                              solaris_pt_result_t *results,
                              bool is_tilt_panel_sensors);

    /**
     * @brief  Read a single sensor by index.
     *
     * @param[in]  handle   Handle from solaris_pt_init().
     * @param[in]  index    Sensor index (0 to num_sensors-1).
     * @param[out] result   Pointer to a single solaris_pt_result_t.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if index out of range.
     */
    esp_err_t solaris_pt_read_single(solaris_pt_handle_t handle,
                                     int index,
                                     solaris_pt_result_t *result);

    /**
     * @brief  Log all sensor results at INFO level.
     */
    void solaris_pt_log(solaris_pt_handle_t handle,
                        const solaris_pt_result_t *results);

    /**
     * @brief  Free all resources and invalidate the handle.
     */
    esp_err_t solaris_pt_deinit(solaris_pt_handle_t handle);

    void solar_tracking(void *pvParameters);

    extern solaris_pt_config_t pt_cfg;
    extern solaris_pt_handle_t pt;

#ifdef __cplusplus
}
#endif