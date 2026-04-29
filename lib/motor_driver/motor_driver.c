#include "motor_driver.h"
// panel_state.c
static volatile panel_limit_state_t s_state = PANEL_OK;

panel_limit_state_t panel_get_limit_state(void) {
    return s_state;
}

void panel_set_limit_state(panel_limit_state_t state) {
    s_state = state;
}