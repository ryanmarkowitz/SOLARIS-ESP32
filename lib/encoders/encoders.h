#ifndef ENCODERS_H
#define ENCODERS_H

/* Includes */
#include "driver/pulse_cnt.h"
#include "esp_err.h"

/* Function declarations */
void encoder_init();
esp_err_t save_position_to_flash();

#endif