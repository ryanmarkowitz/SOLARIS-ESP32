#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


// panel_state.h
typedef enum {
    PANEL_OK,                  // free to move either direction
    PANEL_AT_UPPER_LIMIT,      // can only move down
    PANEL_AT_LOWER_LIMIT,      // can only move up
} panel_limit_state_t;

panel_limit_state_t panel_get_limit_state(void);
void panel_set_limit_state(panel_limit_state_t state);

#endif