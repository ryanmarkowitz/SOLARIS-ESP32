/**
 * @file    solaris_icm20948.h
 * @brief   SOLARIS ICM-20948 9-DoF IMU Library
 *
 * Communicates via I2C. Supports reading raw accelerometer and gyroscope data,
 * and configures the hardware interrupt pin for data-ready signalling.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ---------------------------------------------------------------------------
    // Configuration struct
    // ---------------------------------------------------------------------------

    /**
     * @brief  Configuration passed to solaris_icm20948_init().
     */
    typedef struct
    {
        /* I2C Bus Configuration */
        int i2c_scl;         /**< I2C Clock Pin (SCL) */
        int i2c_sda;         /**< I2C Data Pin (SDA) */
        i2c_port_t i2c_port; /**< I2C Hardware Port (e.g., I2C_NUM_0) */
        uint32_t i2c_freq;   /**< I2C Clock Frequency in Hz */
        uint8_t i2c_addr;    /**< Device I2C Address (0x69 default, 0x68 alternate) */

        /* GPIO Configuration */
        int int_gpio; /**< Interrupt pin from IMU (Data Ready) */
    } solaris_icm20948_config_t;

/**
 * @brief  Sensible defaults matching the user's specific wiring.
 */
#define SOLARIS_ICM20948_CONFIG_DEFAULT() { \
    .i2c_scl = 40,                          \
    .i2c_sda = 41,                          \
    .i2c_port = I2C_NUM_0,                  \
    .i2c_freq = 400000,                     \
    .i2c_addr = 0x69,                       \
    .int_gpio = 39,                         \
}

    // ---------------------------------------------------------------------------
    // Handle
    // ---------------------------------------------------------------------------

    /** Opaque handle returned by solaris_icm20948_init(). */
    typedef struct solaris_icm_ctx_t *solaris_icm20948_handle_t;

    // ---------------------------------------------------------------------------
    // Measurement result
    // ---------------------------------------------------------------------------

    /**
     * @brief  Measurement result structure for 6-DoF data.
     */
    typedef struct
    {
        // Accelerometer (Raw)
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;

        // Gyroscope (Raw)
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;

        // Magnetometer / Compass (Raw)
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    } solaris_icm20948_result_t;
    // ---------------------------------------------------------------------------
    // API
    // ---------------------------------------------------------------------------

    /**
     * @brief  Initialise the ICM-20948 sensor subsystem.
     *
     * Configures I2C, wakes the sensor, verifies the WHO_AM_I register,
     * and configures the data-ready interrupt pin.
     *
     * @param[in]  config  Pointer to a populated solaris_icm20948_config_t.
     * @param[out] handle  Receives the allocated context handle on success.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t solaris_icm20948_init(const solaris_icm20948_config_t *config,
                                    solaris_icm20948_handle_t *handle);

    /**
     * @brief  Check if the IMU interrupt pin is high (data ready).
     *
     * @param[in]  handle  Handle returned by solaris_icm20948_init().
     * @return true if data is ready, false otherwise.
     */
    bool solaris_icm20948_data_ready(solaris_icm20948_handle_t handle);

    /**
     * @brief  Read the raw Accelerometer and Gyroscope data.
     *
     * @param[in]  handle  Handle returned by solaris_icm20948_init().
     * @param[out] result  Pointer to a solaris_icm20948_result_t to populate.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t solaris_icm20948_read(solaris_icm20948_handle_t handle,
                                    solaris_icm20948_result_t *result);

    /**
     * @brief  Re-zero the gyro bias using fresh stationary samples.
     *
     * Call immediately before a maneuver that integrates gyro readings
     * (e.g. motor_turn_degrees), while the chassis is still stationary, to
     * correct for bias drift since solaris_icm20948_init() (or the last
     * recalibration) ran. Updates the bias applied by every subsequent
     * solaris_icm20948_read() call.
     *
     * @param[in]  handle       Handle returned by solaris_icm20948_init().
     * @param[in]  num_samples  Number of samples to average (~10ms apart).
     */
    void solaris_icm20948_recalibrate_gyro(solaris_icm20948_handle_t handle, int num_samples);

    /**
     * @brief  Log the 6-DoF results at INFO level.
     *
     * @param[in]  result  Result populated by solaris_icm20948_read().
     */
    void solaris_icm20948_log(const solaris_icm20948_result_t *result);

    /**
     * @brief  Free all resources and invalidate the handle.
     *
     * @param[in]  handle  Handle to deinitialise.
     * @return ESP_OK on success.
     */
    esp_err_t solaris_icm20948_deinit(solaris_icm20948_handle_t handle);

    extern solaris_icm20948_config_t imu_cfg;
    extern solaris_icm20948_handle_t imu_handle;

#ifdef __cplusplus
}
#endif