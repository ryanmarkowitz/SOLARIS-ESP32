#include "solaris_manual_ctrl.h"

static int8_t g_throttle = 0;
static int8_t g_steering = 0;

void solaris_manual_ctrl_set(int8_t throttle, int8_t steering)
{
    g_throttle = throttle;
    g_steering = steering;
}

void solaris_manual_ctrl_get(int8_t *throttle_out, int8_t *steering_out)
{
    if (throttle_out)
        *throttle_out = g_throttle;
    if (steering_out)
        *steering_out = g_steering;
}
