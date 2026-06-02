#ifndef ENCODERS_H
#define ENCODERS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/pulse_cnt.h"

#define NUM_ENCODERS 4
#define PAN_ENCODER_ID 0
#define TILT_ENCODER_ID 1
#define FL_ENCODER_ID 2
#define FR_ENCODER_ID 3

/* Structs */
typedef struct
{
    uint8_t encoder_gpio;
    int8_t dir_gpio;
} encoder_pins_t;

typedef struct
{
    encoder_pins_t pins;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t channel_handle;
} encoder_t;

typedef struct
{
    uint8_t id;
    QueueHandle_t queue;
    int8_t dir_gpio;
} encoder_ctxt_t;

typedef struct
{
    uint8_t id;
    int16_t watch_point_value;
    uint8_t dir_level;
} encoder_evt_t;

/* Includes */
#include "driver/pulse_cnt.h"
#include "esp_err.h"

/* Function declarations */
void encoder_init();
esp_err_t save_position_to_flash(uint8_t encoder_id);
int get_pulse_count(uint8_t encoder_id);
int get_distance_traveled();

#endif