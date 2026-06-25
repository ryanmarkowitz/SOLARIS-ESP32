#pragma once
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

/* Variable definitions */

// ADC Channel 1 definitions (GPIOs connected to the sensors)
#define SENSOR_X1_CHANNEL ADC_CHANNEL_0 // GPIO1
#define SENSOR_X2_CHANNEL ADC_CHANNEL_1 // GPIO2
#define SENSOR_Y1_CHANNEL ADC_CHANNEL_2 // GPIO3
#define SENSOR_Y2_CHANNEL ADC_CHANNEL_4 // GPIO5

// ADC Channel 1 Configuration
#define ADC_ATTENUATION ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_handle;

/* Function Declarations */
void solar_tracking(void);