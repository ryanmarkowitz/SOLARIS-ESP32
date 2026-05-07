/**
 * @file    solaris_ultrasonic.h
 * @brief   SOLARIS Ultrasonic Sensor Library
 *
 * Supports up to 4x MB1020 analog ultrasonic sensors multiplexed through a
 * CD4052BE 4-channel analog mux, sampled via ESP32 ADC (oneshot driver).
 *
 * Wiring summary:
 *   - All sensor TRIG pins → single shared GPIO (SOLARIS_US_TRIGGER_GPIO)
 *   - All sensor AN pins  → CD4052BE inputs (X0–X3 or Y0–Y3)
 *   - CD4052BE common out → ESP32 ADC pin
 *   - CD4052BE SEL_A      → SOLARIS_US_MUX_SEL_A
 *   - CD4052BE SEL_B      → SOLARIS_US_MUX_SEL_B
 *   - CD4052BE INH (pin 6)→ GND
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/** Maximum number of sensors the library supports. */
#define SOLARIS_US_MAX_SENSORS  4

// ---------------------------------------------------------------------------
// Configuration struct
// ---------------------------------------------------------------------------

/**
 * @brief  Configuration passed to solaris_us_init().
 *
 * All fields must be set before calling init. Defaults are provided via
 * SOLARIS_US_CONFIG_DEFAULT() for the most common wiring.
 */
typedef struct {
    /* GPIO */
    int trigger_gpio;   /**< Shared trigger output pin (active-high pulse). */
    int mux_sel_a;      /**< CD4052BE SEL_A (pin 9)  — bit 0 of channel select. */
    int mux_sel_b;      /**< CD4052BE SEL_B (pin 10) — bit 1 of channel select. */

    /* ADC */
    adc_unit_t        adc_unit;     /**< ADC unit (ADC_UNIT_1 recommended). */
    adc_channel_t     adc_channel;  /**< ADC channel connected to mux output. */
    adc_atten_t       adc_atten;    /**< Attenuation (ADC_ATTEN_DB_12 for 0–3.3 V). */
    adc_bitwidth_t    adc_bitwidth; /**< Bit width (ADC_BITWIDTH_12). */
    int               adc_samples;  /**< Averaged samples per read (e.g. 32). */

    /* Sensor count */
    int num_sensors;    /**< Number of sensors in use (1–SOLARIS_US_MAX_SENSORS). */

    /* MB1020 scaling */
    int vcc_mv;         /**< Supply voltage in millivolts (typically 3300). */

    /* Timing (microseconds / milliseconds) */
    int trigger_pulse_us;   /**< Trigger pulse width in µs (≥20 µs, typically 30). */
    int trigger_settle_us;  /**< Post-trigger settle time in µs (typically 100). */
    int mux_settle_us;      /**< Mux channel switch settle time in µs (typically 50). */
    int ranging_delay_ms;   /**< Wait after trigger for ranging to complete (ms). */
} solaris_us_config_t;

/**
 * @brief  Sensible defaults matching the reference schematic.
 *
 * Override individual fields before calling solaris_us_init().
 *
 * Example:
 * @code
 *   solaris_us_config_t cfg = SOLARIS_US_CONFIG_DEFAULT();
 *   cfg.num_sensors = 2;
 *   solaris_us_init(&cfg, &handle);
 * @endcode
 */
#define SOLARIS_US_CONFIG_DEFAULT() {           \
    .trigger_gpio      = 20,                    \
    .mux_sel_a         = 2,                     \
    .mux_sel_b         = 3,                     \
    .adc_unit          = ADC_UNIT_1,            \
    .adc_channel       = ADC_CHANNEL_0,         \
    .adc_atten         = ADC_ATTEN_DB_12,       \
    .adc_bitwidth      = ADC_BITWIDTH_12,       \
    .adc_samples       = 32,                    \
    .num_sensors       = SOLARIS_US_MAX_SENSORS,\
    .vcc_mv            = 3300,                  \
    .trigger_pulse_us  = 30,                    \
    .trigger_settle_us = 100,                   \
    .mux_settle_us     = 50,                    \
    .ranging_delay_ms  = 100,                   \
}

// ---------------------------------------------------------------------------
// Handle
// ---------------------------------------------------------------------------

/** Opaque handle returned by solaris_us_init(). */
typedef struct solaris_us_ctx_t *solaris_us_handle_t;

// ---------------------------------------------------------------------------
// Measurement result
// ---------------------------------------------------------------------------

/**
 * @brief  Per-sensor measurement result.
 */
typedef struct {
    int   raw;      /**< Raw 12-bit ADC reading. */
    int   mv;       /**< Voltage in millivolts (calibrated if available). */
    float inches;   /**< Distance in inches. */
    float cm;       /**< Distance in centimetres. */
} solaris_us_result_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise the SOLARIS ultrasonic sensor subsystem.
 *
 * Configures GPIO, ADC unit, channel, and optional curve-fitting calibration.
 * Must be called once before any other solaris_us_* function.
 *
 * @param[in]  config   Pointer to a populated solaris_us_config_t.
 * @param[out] handle   Receives the allocated context handle on success.
 *
 * @return ESP_OK on success, or an esp_err_t error code.
 */
esp_err_t solaris_us_init(const solaris_us_config_t *config,
                          solaris_us_handle_t       *handle);

/**
 * @brief  Trigger all sensors and read distances.
 *
 * Fires a single shared trigger pulse, waits for ranging to complete, then
 * cycles through each mux channel and populates @p results.
 *
 * @param[in]  handle   Handle returned by solaris_us_init().
 * @param[out] results  Array of at least config->num_sensors elements.
 *
 * @return ESP_OK on success, or an esp_err_t error code.
 */
esp_err_t solaris_us_read(solaris_us_handle_t  handle,
                          solaris_us_result_t *results);

/**
 * @brief  Read a single sensor by index without re-triggering.
 *
 * Useful when you need to poll one channel after a shared trigger has already
 * been fired. Does NOT issue a trigger pulse.
 *
 * @param[in]  handle   Handle returned by solaris_us_init().
 * @param[in]  index    Sensor index (0 to num_sensors-1).
 * @param[out] result   Pointer to a single solaris_us_result_t.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if index out of range.
 */
esp_err_t solaris_us_read_single(solaris_us_handle_t  handle,
                                 int                  index,
                                 solaris_us_result_t *result);

/**
 * @brief  Fire the shared trigger pulse only (no ADC read).
 *
 * Use together with solaris_us_read_single() when you need manual control
 * over trigger timing.
 *
 * @param[in]  handle   Handle returned by solaris_us_init().
 *
 * @return ESP_OK on success.
 */
esp_err_t solaris_us_trigger(solaris_us_handle_t handle);

/**
 * @brief  Log all sensor results at INFO level.
 *
 * Prints both inch and centimetre values for each sensor using ESP_LOGI.
 *
 * @param[in]  handle   Handle returned by solaris_us_init().
 * @param[in]  results  Array populated by solaris_us_read().
 */
void solaris_us_log(solaris_us_handle_t        handle,
                    const solaris_us_result_t *results);

/**
 * @brief  Print Teleplot/Serial-Plotter compatible output to stdout.
 *
 * Format: >S0:<val> >S1:<val> ... \n
 *
 * @param[in]  handle   Handle returned by solaris_us_init().
 * @param[in]  results  Array populated by solaris_us_read().
 */
void solaris_us_print_teleplot(solaris_us_handle_t        handle,
                               const solaris_us_result_t *results);

/**
 * @brief  Free all resources and invalidate the handle.
 *
 * @param[in]  handle   Handle to deinitialise.
 *
 * @return ESP_OK on success.
 */
esp_err_t solaris_us_deinit(solaris_us_handle_t handle);

#ifdef __cplusplus
}
#endif