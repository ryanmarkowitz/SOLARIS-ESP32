#include "solaris_icm20948.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "shared_resources.h"

static const char *TAG = "SOLARIS_IMU";

// --- ICM-20948 Registers (Bank 0) ---
#define ICM20948_WHO_AM_I 0x00
#define ICM20948_WHO_AM_I_VAL 0xEA
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_INT_PIN_CFG 0x0F
#define ICM20948_INT_ENABLE 0x10
#define ICM20948_ACCEL_XOUT_H 0x2D

// --- AK09916 Magnetometer Registers ---
#define AK09916_I2C_ADDR 0x0C
#define AK09916_WIA2 0x01
#define AK09916_WIA2_VAL 0x09
#define AK09916_ST1 0x10
#define AK09916_HXL 0x11
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32

// ---------------------------------------------------------------------------

struct solaris_icm_ctx_t
{
    solaris_icm20948_config_t cfg;
    bool i2c_installed_by_us;
};

// ---------------------------------------------------------------------------
// Private helpers (Now accepting target dev_addr)
// ---------------------------------------------------------------------------

static esp_err_t prv_write_register(const struct solaris_icm_ctx_t *ctx, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_to_device(ctx->cfg.i2c_port, dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
    xSemaphoreGive(i2c_bus_mutex);
    return err;
}

static esp_err_t prv_read_registers(const struct solaris_icm_ctx_t *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_write_read_device(ctx->cfg.i2c_port, dev_addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
    xSemaphoreGive(i2c_bus_mutex);
    return err;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t solaris_icm20948_init(const solaris_icm20948_config_t *config, solaris_icm20948_handle_t *handle)
{
    if (!config || !handle)
        return ESP_ERR_INVALID_ARG;

    struct solaris_icm_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
        return ESP_ERR_NO_MEM;

    ctx->cfg = *config;

    // --- 2. GPIO Interrupt Pin ---
    if (ctx->cfg.int_gpio >= 0)
    {
        gpio_reset_pin(ctx->cfg.int_gpio);
        gpio_set_direction(ctx->cfg.int_gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(ctx->cfg.int_gpio, GPIO_FLOATING);
    }

    // --- 3. Wake & Verify ICM-20948 ---
    uint8_t who_am_i = 0;
    esp_err_t err = prv_read_registers(ctx, ctx->cfg.i2c_addr, ICM20948_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK || who_am_i != ICM20948_WHO_AM_I_VAL)
    {
        ESP_LOGE(TAG, "ICM-20948 WHO_AM_I failed (ID: 0x%02X).", who_am_i);
        free(ctx);
        return ESP_FAIL;
    }

    // Clear sleep bit
    prv_write_register(ctx, ctx->cfg.i2c_addr, ICM20948_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // INT Pin Config: 0x32 (Clear on Read + Bypass Enable)
    prv_write_register(ctx, ctx->cfg.i2c_addr, ICM20948_INT_PIN_CFG, 0x32);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable Raw Data Ready Interrupt
    prv_write_register(ctx, ctx->cfg.i2c_addr, ICM20948_INT_ENABLE, 0x01);

    // --- 4. Wake & Verify AK09916 Magnetometer ---
    uint8_t mag_id = 0;
    err = prv_read_registers(ctx, AK09916_I2C_ADDR, AK09916_WIA2, &mag_id, 1);
    if (err != ESP_OK || mag_id != AK09916_WIA2_VAL)
    {
        ESP_LOGW(TAG, "AK09916 Magnetometer not found! Check bypass config.");
    }
    else
    {
        prv_write_register(ctx, AK09916_I2C_ADDR, AK09916_CNTL3, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        prv_write_register(ctx, AK09916_I2C_ADDR, AK09916_CNTL2, 0x08);
    }

    ESP_LOGI(TAG, "9-DoF Initialised successfully.");
    *handle = ctx;
    return ESP_OK;
}
// ---------------------------------------------------------------------------

bool solaris_icm20948_data_ready(solaris_icm20948_handle_t handle)
{
    if (!handle || handle->cfg.int_gpio < 0)
        return false;
    return gpio_get_level(handle->cfg.int_gpio) == 1;
}

// ---------------------------------------------------------------------------

esp_err_t solaris_icm20948_read(solaris_icm20948_handle_t handle, solaris_icm20948_result_t *result)
{
    if (!handle || !result)
        return ESP_ERR_INVALID_ARG;

    uint8_t imu_raw[14];
    uint8_t mag_raw[8];

    // 1. Read ICM-20948 (Big-Endian)
    esp_err_t err = prv_read_registers(handle, handle->cfg.i2c_addr, ICM20948_ACCEL_XOUT_H, imu_raw, 14);
    if (err != ESP_OK)
        return err;

    result->accel_x = (int16_t)((imu_raw[0] << 8) | imu_raw[1]);
    result->accel_y = (int16_t)((imu_raw[2] << 8) | imu_raw[3]);
    result->accel_z = (int16_t)((imu_raw[4] << 8) | imu_raw[5]);

    result->gyro_x = (int16_t)((imu_raw[8] << 8) | imu_raw[9]);
    result->gyro_y = (int16_t)((imu_raw[10] << 8) | imu_raw[11]);
    result->gyro_z = (int16_t)((imu_raw[12] << 8) | imu_raw[13]);

    // 2. Read AK09916 (Little-Endian)
    // We must read 8 bytes starting at HXL. The 8th byte is the ST2 register.
    // Reading ST2 is mandatory—it signals the magnetometer that we are done reading
    // so it can unlock the registers and take the next measurement.
    err = prv_read_registers(handle, AK09916_I2C_ADDR, AK09916_HXL, mag_raw, 8);
    if (err == ESP_OK)
    {
        result->mag_x = (int16_t)((mag_raw[1] << 8) | mag_raw[0]);
        result->mag_y = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
        result->mag_z = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);
    }
    else
    {
        result->mag_x = 0;
        result->mag_y = 0;
        result->mag_z = 0;
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------

void solaris_icm20948_log(const solaris_icm20948_result_t *result)
{
    if (!result)
        return;
    ESP_LOGI(TAG, "A[%6d %6d %6d] | G[%6d %6d %6d] | M[%6d %6d %6d]",
             result->accel_x, result->accel_y, result->accel_z,
             result->gyro_x, result->gyro_y, result->gyro_z,
             result->mag_x, result->mag_y, result->mag_z);
}

// ---------------------------------------------------------------------------

esp_err_t solaris_icm20948_deinit(solaris_icm20948_handle_t handle)
{
    if (!handle)
        return ESP_ERR_INVALID_ARG;
    if (handle->i2c_installed_by_us)
    {
        i2c_driver_delete(handle->cfg.i2c_port);
    }
    free(handle);
    return ESP_OK;
}