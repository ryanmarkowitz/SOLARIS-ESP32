/**
 * @file    solaris_ultrasonic.h
 * @brief   SOLARIS Ultrasonic Sensor Library (LV-MaxSonar-EZ / MB10xx)
 *
 * Up to 4 analog ultrasonic sensors multiplexed through a CD4052BE 4-channel
 * analog mux, sampled via one ESP32 ADC pin (oneshot driver).
 *
 * Wiring summary:
 *   - All sensor RX pins  -> single shared trigger GPIO (trigger_gpio)
 *   - All sensor AN pins  -> CD4052BE channel inputs (through a divider)
 *   - CD4052BE common out -> ESP32 ADC pin
 *   - CD4052BE SEL_A/SEL_B -> mux_sel_a / mux_sel_b
 *   - CD4052BE INH (pin 6)-> GND
 *
 * VOLTAGE PATH (unchanged from the working design):
 *   Sensor AN swings 0..Vcc (Vcc = 5 V) at Vcc/512 per inch. A 1k/2k divider
 *   scales that to ~0..3.3 V for the ADC. The measured pin voltage is scaled
 *   back up by 1.5x in the .c file to recover the true AN voltage, then
 *   divided by (vcc_mv/512) to get inches.
 *
 * TIMING (this is what was fixed):
 *   The AN pin updates once per ~49 ms ranging cycle and RAMPS toward the true
 *   value; one cycle is not enough (datasheet, Timing Description). RX must be
 *   held HIGH across several cycles before sampling. Pulsing RX low stops
 *   ranging, which left the old code sampling a half-ramped, low voltage.
 *
 * CROSS-TALK: one shared trigger ranges all sensors at once, which can
 *   interfere depending on placement. If so, hardware chaining is the fix.
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

/** Maximum number of sensors the library supports (CD4052BE = 4 channels). */
#define SOLARIS_US_MAX_SENSORS 4

    typedef struct
    {
        /* GPIO */
        int trigger_gpio; /**< Shared RX/trigger line to all sensors. */
        int mux_sel_a;    /**< CD4052BE SEL_A (pin 9)  — select bit 0. */
        int mux_sel_b;    /**< CD4052BE SEL_B (pin 10) — select bit 1. */

        /* ADC */
        adc_unit_t adc_unit;         /**< ADC unit (ADC_UNIT_1 recommended). */
        adc_channel_t adc_channel;   /**< ADC channel connected to mux output. */
        adc_atten_t adc_atten;       /**< Attenuation (ADC_ATTEN_DB_12). */
        adc_bitwidth_t adc_bitwidth; /**< Bit width (ADC_BITWIDTH_12). */
        int adc_samples;             /**< Averaged samples per read. */

        /* Sensor count */
        int num_sensors; /**< 1..SOLARIS_US_MAX_SENSORS. */

        /* Scaling (unchanged) */
        int vcc_mv; /**< Sensor supply in mV (5000). Sets vcc_mv/512 mV/in. */

        /* Timing */
        int ranging_cycle_ms; /**< One MB10xx ranging cycle (~49 ms). */
        int settle_cycles;    /**< Cycles to hold RX high before sampling (~3). */
        int mux_settle_us;    /**< Mux switch settle time in µs. */
        bool free_run;        /**< true = RX tied high externally (single sensor);
                                   driver skips triggering and just reads.
                                   Leave false for multi-sensor. */
    } solaris_us_config_t;

/**
 * @brief Defaults for the reference schematic (4 sensors, triggered burst).
 */
#define SOLARIS_US_CONFIG_DEFAULT() {      \
    .trigger_gpio = 20,                    \
    .mux_sel_a = 2,                        \
    .mux_sel_b = 3,                        \
    .adc_unit = ADC_UNIT_1,                \
    .adc_channel = ADC_CHANNEL_0,          \
    .adc_atten = ADC_ATTEN_DB_12,          \
    .adc_bitwidth = ADC_BITWIDTH_12,       \
    .adc_samples = 16,                     \
    .num_sensors = SOLARIS_US_MAX_SENSORS, \
    .vcc_mv = 5000,                        \
    .ranging_cycle_ms = 49,                \
    .settle_cycles = 3,                    \
    .mux_settle_us = 250,                  \
    .free_run = false,                     \
}

    /** Opaque handle returned by solaris_us_init(). */
    typedef struct solaris_us_ctx_t *solaris_us_handle_t;

    /** Per-sensor measurement result. */
    typedef struct
    {
        int raw;      /**< Raw 12-bit ADC reading. */
        int mv;       /**< Voltage in millivolts (calibrated if available). */
        float inches; /**< Distance in inches. */
        float cm;     /**< Distance in centimetres. */
    } solaris_us_result_t;

    esp_err_t solaris_us_init(const solaris_us_config_t *config,
                              solaris_us_handle_t *handle);

    /**
     * @brief Run a ranging burst on all sensors, then read every channel.
     *
     * Holds RX high for settle_cycles ranging cycles so the AN lines settle,
     * samples each mux channel, then stops ranging. In free_run mode it skips
     * the burst and just samples.
     */
    esp_err_t solaris_us_read(solaris_us_handle_t handle,
                              solaris_us_result_t *results);

    /** Read one channel without triggering (AN must already be valid). */
    esp_err_t solaris_us_read_single(solaris_us_handle_t handle,
                                     int index,
                                     solaris_us_result_t *result);

    /** Fire a ranging burst and stop, leaving fresh AN values to read_single. */
    esp_err_t solaris_us_trigger(solaris_us_handle_t handle);

    void solaris_us_log(solaris_us_handle_t handle,
                        const solaris_us_result_t *results);

    void solaris_us_print_teleplot(solaris_us_handle_t handle,
                                   const solaris_us_result_t *results);

    esp_err_t solaris_us_deinit(solaris_us_handle_t handle);

#ifdef __cplusplus
}
#endif