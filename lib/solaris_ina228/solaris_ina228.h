/**
 * @file    solaris_ina228.h
 * @brief   SOLARIS INA228 Power Monitor Library
 *
 * Provides voltage, current, power, energy, and charge monitoring
 * via the Texas Instruments INA228 over I2C.
 *
 * Wiring summary:
 *   - INA228 SDA → ESP32 SDA pin
 *   - INA228 SCL → ESP32 SCL pin
 *   - INA228 A0, A1 → GND (address 0x40)
 *   - INA228 IN+ / IN- → across shunt resistor on battery line
 *   - External I2C pull-up resistors recommended (4.7kΩ to 3.3V)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "shared_resources.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ---------------------------------------------------------------------------
    // Internal register addresses
    // ---------------------------------------------------------------------------

#define INA228_REG_CONFIG 0x00          /**< Device configuration */
#define INA228_REG_ADC_CONFIG 0x01      /**< ADC conversion settings */
#define INA228_REG_SHUNT_CAL 0x02       /**< Shunt calibration */
#define INA228_REG_SHUNT_TEMPCO 0x03    /**< Shunt temperature coefficient */
#define INA228_REG_VSHUNT 0x04          /**< Differential shunt voltage */
#define INA228_REG_VBUS 0x05            /**< Bus voltage */
#define INA228_REG_DIETEMP 0x06         /**< Internal die temperature */
#define INA228_REG_CURRENT 0x07         /**< Calculated current */
#define INA228_REG_POWER 0x08           /**< Calculated power */
#define INA228_REG_ENERGY 0x09          /**< Accumulated energy */
#define INA228_REG_CHARGE 0x0A          /**< Accumulated charge */
#define INA228_REG_DIAG_ALRT 0x0B       /**< Diagnostics and alert */
#define INA228_REG_SOVL 0x0C            /**< Shunt overvoltage limit */
#define INA228_REG_SUVL 0x0D            /**< Shunt undervoltage limit */
#define INA228_REG_BOVL 0x0E            /**< Bus overvoltage limit */
#define INA228_REG_BUVL 0x0F            /**< Bus undervoltage limit */
#define INA228_REG_TEMP_LIMIT 0x10      /**< Temperature over limit */
#define INA228_REG_PWR_LIMIT 0x11       /**< Power over limit */
#define INA228_REG_MANUFACTURER_ID 0x3E /**< Should return 0x5449 */
#define INA228_REG_DEVICE_ID 0x3F       /**< Should return 0x2281 */

#define SOLARIS_RING_BUFFER_SIZE 180 // every 1s read power from the energy monitor unit and store value in buffer

    extern float solaris_power_buffer[SOLARIS_RING_BUFFER_SIZE];
    extern int solaris_power_buffer_idx;
    extern float solaris_power_buffer_with_moves_included[60];
    extern int solaris_power_buffer_with_moves_included_idx;

    // ---------------------------------------------------------------------------
    // Configuration struct
    // ---------------------------------------------------------------------------

    /**
     * @brief  Configuration passed to solaris_ina228_init().
     */
    typedef struct
    {
        /* I2C */
        i2c_port_t i2c_port;   /**< I2C port number (I2C_NUM_0 or I2C_NUM_1). */
        int sda_io;            /**< SDA GPIO pin. */
        int scl_io;            /**< SCL GPIO pin. */
        uint32_t clk_speed_hz; /**< I2C clock speed in Hz (typically 400000). */
        uint32_t timeout_ms;   /**< I2C transaction timeout in ms. */

        /* INA228 device */
        uint8_t i2c_addr; /**< I2C address (0x40–0x4F based on A0/A1 pins). */

        /* Calibration */
        float current_lsb;  /**< Current LSB in amps (e.g. 1e-6 for 1µA resolution). */
        uint16_t shunt_cal; /**< SHUNT_CAL register value. */

        /* Register config values */
        uint16_t config_reg;     /**< Value to write to CONFIG register. */
        uint16_t adc_config_reg; /**< Value to write to ADC_CONFIG register. */

        /* Battery */
        float battery_capacity_mah; /**< Battery capacity in mAh for SOC calculation. */
    } solaris_ina228_config_t;

/**
 * @brief  Default configuration matching the SOLARIS reference design.
 *
 * Override individual fields before calling solaris_ina228_init().
 *
 * Example:
 * @code
 *   solaris_ina228_config_t cfg = SOLARIS_INA228_CONFIG_DEFAULT();
 *   cfg.battery_capacity_mah = 5000.0f;
 *   solaris_ina228_init(&cfg, &handle);
 * @endcode
 */
#define SOLARIS_INA228_CONFIG_DEFAULT() { \
    .i2c_port = I2C_NUM_0,                \
    .sda_io = 41,                         \
    .scl_io = 40,                         \
    .clk_speed_hz = 400000,               \
    .timeout_ms = 1000,                   \
    .i2c_addr = 0x40,                     \
    .current_lsb = 1.0e-6f,               \
    .shunt_cal = 1049,                    \
    .config_reg = 0x0010,                 \
    .adc_config_reg = 0xFB6A,             \
    .battery_capacity_mah = 5000.0f,      \
}

    // ---------------------------------------------------------------------------
    // Handle
    // ---------------------------------------------------------------------------

    // ---------------------------------------------------------------------------
    // Measurement result
    // ---------------------------------------------------------------------------

    /**
     * @brief  Full snapshot of all INA228 measurements.
     */
    typedef struct
    {
        float voltage_v;     /**< Bus voltage in volts. */
        float current_a;     /**< Current in amps (negative = charging). */
        float current_ma;    /**< Current in milliamps. */
        float power_w;       /**< Power in watts. */
        float energy_j;      /**< Accumulated energy in joules. */
        float charge_c;      /**< Accumulated charge in coulombs. */
        float charge_mah;    /**< Accumulated charge in milliamp-hours. */
        float temperature_c; /**< Die temperature in degrees Celsius. */
        uint8_t soc_percent; /**< Estimated state of charge (0–100%). */
    } solaris_ina228_result_t;

    // ---------------------------------------------------------------------------
    // API
    // ---------------------------------------------------------------------------

    /**
     * @brief  Initialise the INA228 and verify the device is present.
     *
     * Configures I2C, writes CONFIG, ADC_CONFIG, and SHUNT_CAL registers,
     * and verifies the manufacturer ID.
     *
     * @param[in]  config   Pointer to a populated solaris_ina228_config_t.
     * @param[out] handle   Receives the allocated context handle on success.
     *
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not detected,
     *         or another esp_err_t on failure.
     */
    esp_err_t solaris_ina228_init(const solaris_ina228_config_t *config,
                                  solaris_ina228_handle_t *handle);

    /**
     * @brief  Read all measurements in a single call.
     *
     * Reads VBUS, CURRENT, POWER, ENERGY, CHARGE, and DIETEMP registers
     * and populates the result struct including SOC estimate.
     *
     * @param[in]  handle   Handle returned by solaris_ina228_init().
     * @param[out] result   Pointer to a solaris_ina228_result_t to populate.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t solaris_ina228_read(solaris_ina228_handle_t handle,
                                  solaris_ina228_result_t *result);

    /**
     * @brief  Read bus voltage only.
     *
     * @param[in]  handle     Handle returned by solaris_ina228_init().
     * @param[out] voltage_v  Bus voltage in volts.
     *
     * @return ESP_OK on success.
     */
    esp_err_t solaris_ina228_read_voltage(solaris_ina228_handle_t handle,
                                          float *voltage_v);

    /**
     * @brief  Read current only.
     *
     * @param[in]  handle     Handle returned by solaris_ina228_init().
     * @param[out] current_a  Current in amps.
     *
     * @return ESP_OK on success.
     */
    esp_err_t solaris_ina228_read_current(solaris_ina228_handle_t handle,
                                          float *current_a);

    /**
     * @brief  Read state of charge estimate (0–100%).
     *
     * Uses coulomb counting from the CHARGE register, anchored to the
     * last SOC value persisted to flash (see set_soc()).
     *
     * @param[in]  handle       Handle returned by solaris_ina228_init().
     * @param[out] soc_percent  State of charge percentage.
     *
     * @return ESP_OK on success.
     */
    esp_err_t solaris_ina228_read_soc(solaris_ina228_handle_t handle,
                                      uint8_t *soc_percent);

    /**
     * @brief  Reset the energy and charge accumulation registers.
     *
     * Writes to CONFIG register with the RSTACC bit set.
     * Call this when starting a new charge/discharge cycle.
     *
     * @param[in]  handle   Handle returned by solaris_ina228_init().
     *
     * @return ESP_OK on success.
     */
    esp_err_t solaris_ina228_reset_accumulators(solaris_ina228_handle_t handle);

    /**
     * @brief  Log all measurements at INFO level.
     *
     * @param[in]  handle   Handle returned by solaris_ina228_init().
     * @param[in]  result   Result populated by solaris_ina228_read().
     */
    void solaris_ina228_log(solaris_ina228_handle_t handle,
                            const solaris_ina228_result_t *result);

    /**
     * @brief  Print Teleplot/Serial-Plotter compatible output to stdout.
     *
     * Format: >V:<val> >I:<val> >P:<val> >SOC:<val>\n
     *
     * @param[in]  handle   Handle returned by solaris_ina228_init().
     * @param[in]  result   Result populated by solaris_ina228_read().
     */
    void solaris_ina228_print_teleplot(solaris_ina228_handle_t handle,
                                       const solaris_ina228_result_t *result);

    /**
     * @brief  Free all resources and invalidate the handle.
     *
     * @param[in]  handle   Handle to deinitialise.
     *
     * @return ESP_OK on success.
     */
    esp_err_t solaris_ina228_deinit(solaris_ina228_handle_t handle);

    void solaris_ina228_1s_read(void *pvParmaters);
    void solaris_ina228_make_move_decision(void *pvParameters);
    void set_soc(uint8_t soc);

#ifdef __cplusplus
}
#endif