#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"

// Forward-declared rather than including solaris_icm20948.h here: that
// header pulls in the IMU library, and since motor_driver.h is included
// by unrelated libs (e.g. encoders), a full include leaks the dependency
// to consumers that don't need it and confuses PlatformIO's LDF. This is
// the same opaque-pointer type from solaris_icm20948.h — the underlying
// struct is never defined in any header, only in solaris_icm20948.c.
typedef struct solaris_icm_ctx_t *solaris_icm20948_handle_t;

#define NUM_MOTORS 4
#define PANEL_PAN_ID 0
#define PANEL_TILT_ID 1
#define MOTOR_PAN_ID 0
#define MOTOR_TILT_ID 1
#define MOTOR_LEFT_ID 2
#define MOTOR_RIGHT_ID 3

// ICM-20948 default gyro full-scale is +/-250 dps -> 131 LSB per dps.
// If GYRO_CONFIG_1 in solaris_icm20948.c is ever changed to a wider FS
// range, this must be updated to match.
#define IMU_GYRO_SENS_LSB_PER_DPS 131.0f

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

/*
 * Rotates SOLARIS in place using the IMU gyro to track how far it has
 * turned. Positive degrees turns right (clockwise, both drive motors
 * backward on this chassis); negative turns left (both forward). Range
 * is clamped to [-360, 360]. Blocks the calling task until the turn
 * completes (or times out). Caller must already hold actuator_mutex.
 */
void motor_turn_degrees(solaris_icm20948_handle_t imu, float degrees);

#endif