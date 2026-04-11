#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

const static char *TAG = "ULTRASONIC_ANALOG";

// --- Configuration ---
#define SONAR_TRIGGER_GPIO 40
#define SONAR_1_CHANNEL ADC_CHANNEL_0 // GPIO1 (Connect to Sensor 1 Pin 3 AN)
#define SONAR_2_CHANNEL ADC_CHANNEL_1 // GPIO2 (Connect to Sensor 2 Pin 3 AN)
#define ADC_ATTENUATION      ADC_ATTEN_DB_12 
#define ADC_BITWIDTH         ADC_BITWIDTH_12

// Scaling math from datasheet: (Vcc / 512) per inch
// If you power the sensor with 3.3V: 3300mV / 512 = ~6.44mV per inch
#define VCC_MV 3300
#define MV_PER_INCH          (VCC_MV/512.0)


void init_sonar_trigger() {
    gpio_reset_pin(SONAR_TRIGGER_GPIO);
    gpio_set_direction(SONAR_TRIGGER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SONAR_TRIGGER_GPIO, 0);
}

void trigger_sonar_reading() {
    gpio_set_level(SONAR_TRIGGER_GPIO, 1);
    esp_rom_delay_us(30);
    gpio_set_level(SONAR_TRIGGER_GPIO, 0);
}

void app_main(void) {
    // 1. Initialize ADC Unit
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // 2. Configure the Channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTENUATION,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SONAR_1_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SONAR_2_CHANNEL, &config));

    // 3. Setup Calibration (To get accurate millivolts)
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH,
        .chan = SONAR_1_CHANNEL,
    };
   bool cali_enabled = (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK);

   init_sonar_trigger();


    while (1) {
        int raw_1 = 0, raw_2 = 0;
        int mv_1 = 0, mv_2 = 0;

        trigger_sonar_reading();
        vTaskDelay(pdMS_TO_TICKS(50));

        // Read both sensors
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, SONAR_1_CHANNEL, &raw_1));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, SONAR_2_CHANNEL, &raw_2));

        if (cali_enabled) {
            adc_cali_raw_to_voltage(cali_handle, raw_1, &mv_1);
            adc_cali_raw_to_voltage(cali_handle, raw_2, &mv_2);

            float inches_1 = (float)mv_1 / MV_PER_INCH;
            float inches_2 = (float)mv_2 / MV_PER_INCH;
            
            // Output for Teleplot / Serial Plotter
            printf(">Sonar1_In:%.2f >Sonar2_In:%.2f\n", inches_1, inches_2);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}