#include "solaris_mode.h"

// global variable to hold current mode. Defualt is set to automatic
static solaris_mode_t g_mode = SOLARIS_MODE_AUTOMATIC;

// get 8 int response from BLE and set the current mode for g_mode
bool solaris_mode_set_from_u8(uint8_t raw_mode)
{
    if (raw_mode > SOLARIS_MODE_MANUAL)
    {
        return false;
    }
    g_mode = (solaris_mode_t)raw_mode;
    return true;
}

solaris_mode_t solaris_mode_get(void) { return g_mode; }

// For debugging to read what mode we are currently in
const char *solaris_mode_to_str(solaris_mode_t mode)
{
    switch (mode)
    {
    case SOLARIS_MODE_STATIONARY:
        return "Stationary";
    case SOLARIS_MODE_AUTOMATIC:
        return "Automatic";
    case SOLARIS_MODE_MANUAL:
        return "Manual";
    default:
        return "Unknown";
    }
}
