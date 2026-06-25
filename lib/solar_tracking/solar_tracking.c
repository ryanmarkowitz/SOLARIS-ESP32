#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "solar_tracking.h"
#include <motor_driver.h>
#include <stdlib.h>

/*
X1 Y1
X2 Y2
*/

#define VOLTAGE_TOLERANCE 10

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_handle;

const static char *TAG = "SOLAR_ADC";

bool init_solar_adc(void)
{
    ESP_LOGI(TAG, "Initializing ADC...");

    // Initialize ADC
    adc_oneshot_unit_handle_t current_adc_handle;
    adc_oneshot_unit_init_cfg_t solartracking_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE, // We aren't using the Ultra Low Power co-processor here
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&solartracking_config, &current_adc_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH, // set to 13-bit (0-8191)
        .atten = ADC_ATTENUATION,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(current_adc_handle, SENSOR_X1_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(current_adc_handle, SENSOR_X2_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(current_adc_handle, SENSOR_Y1_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(current_adc_handle, SENSOR_Y2_CHANNEL, &config));

    // Calibration for ADC Channel
    adc_cali_handle_t current_cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = SENSOR_X1_CHANNEL, // Calibration applies to the unit/attenuation, we just pass one channel to generate the profile
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH,
    };

    bool calibration_success = false;
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &current_cali_handle) == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC Calibration setup successful.");
        calibration_success = true;
    }
    else
    {
        ESP_LOGE(TAG, "ADC Calibration failed! We will only get raw data.");
    }

    adc1_handle = current_adc_handle;
    cali_handle = current_cali_handle;

    return calibration_success;
}

void solar_tracking(void)
{
    int raw_val_x1 = 0,
        raw_val_x2 = 0,
        raw_val_y1 = 0,
        raw_val_y2 = 0;

    int sum_raw_x1 = 0,
        sum_raw_x2 = 0,
        sum_raw_y1 = 0,
        sum_raw_y2 = 0;

    int voltage_x1 = 0,
        voltage_x2 = 0,
        voltage_y1 = 0,
        voltage_y2 = 0;

    int left_right = 0,
        top_down = 0;
    while (1)
    {
        // Read Raw Values (0 to 4095)
        for (int i = 0; i <= 4; i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SENSOR_X1_CHANNEL, &raw_val_x1));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SENSOR_X2_CHANNEL, &raw_val_x2));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SENSOR_Y1_CHANNEL, &raw_val_y1));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SENSOR_Y2_CHANNEL, &raw_val_y2));

            sum_raw_x1 += raw_val_x1;
            sum_raw_x2 += raw_val_x2;
            sum_raw_y1 += raw_val_y1;
            sum_raw_y2 += raw_val_y2;
        }

        sum_raw_x1 /= 5; // Average of 5 readings
        sum_raw_x2 /= 5;
        sum_raw_y1 /= 5;
        sum_raw_y2 /= 5;

        // Convert to Millivolts (mV) using the calibration profile
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, sum_raw_x1, &voltage_x1));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, sum_raw_x2, &voltage_x2));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, sum_raw_y1, &voltage_y1));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, sum_raw_y2, &voltage_y2));

        // LEFT RIGHT CHECK
        left_right = (voltage_x1 + voltage_x2) - (voltage_y1 + voltage_y2);
        if (left_right > VOLTAGE_TOLERANCE)
        { // The panel needs to rotate left
            motor_go_forward(PANEL_PAN_ID, .25);
        }
        else if (abs(left_right) > VOLTAGE_TOLERANCE)
        { // The panel needs to rotate right
            motor_go_backward(PANEL_PAN_ID, .25);
        }
        else
        { // Found correct spot stop panning the motor
            stop_motor(PANEL_PAN_ID);
        }

        // LEFT RIGHT CHECK
        top_down = (voltage_x1 + voltage_y1) - (voltage_x1 + voltage_y2);
        if (top_down > VOLTAGE_TOLERANCE)
        { // The panel needs to rotate left
            motor_go_forward(PANEL_TILT_ID, .25);
        }
        else if (abs(top_down) > VOLTAGE_TOLERANCE)
        { // The panel needs to rotate right
            motor_go_backward(PANEL_TILT_ID, .25);
        }
        else
        { // Found correct spot stop panning the motor
            stop_motor(PANEL_TILT_ID);
        }

        // Wait 1000ms before reading again
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Cleanup ADC resources

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (cali_handle)
    {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali_handle));
    }
}