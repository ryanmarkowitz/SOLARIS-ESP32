#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"

#define NUM_MOTORS 6
#define PANEL_PAN_ID 0
#define PANEL_TILT_ID 1
#define MOTOR_PAN_ID 0
#define MOTOR_TILT_ID 1
#define MOTOR_FL_ID 2
#define MOTOR_FR_ID 3
#define MOTOR_RL_ID 4
#define MOTOR_RR_ID 5

/* Structs and enums */
typedef struct
{
    uint8_t pwm_gpio;
    uint8_t dir_gpio;
} motor_pins_t;

// state for tracking if pan motor tilted too far
typedef enum
{
    PANEL_OK,             // free to move either direction
    PANEL_AT_UPPER_LIMIT, // can only move down
    PANEL_AT_LOWER_LIMIT, // can only move up
} panel_limit_state_t;

typedef struct
{
    motor_pins_t pins;
    mcpwm_cmpr_handle_t comparator; // independent duty per motor
    mcpwm_gen_handle_t generator;   // independent GPIO per motor
    volatile panel_limit_state_t limit_state;
} motor_t;

/* Function Declarations */
panel_limit_state_t panel_get_limit_state(uint8_t panel_id);
void panel_set_limit_state(uint8_t panel_id, panel_limit_state_t state);
void stop_motor(uint8_t panel_id);
void motor_init();
void motor_go_forward(uint8_t panel_id, float duty_cycle);
void motor_go_backward(uint8_t panel_id, float duty_cycle);
void test_motor(void *pvParameters);

#endif