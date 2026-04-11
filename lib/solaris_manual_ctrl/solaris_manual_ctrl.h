#ifndef SOLARIS_MANUAL_CTRL_H
#define SOLARIS_MANUAL_CTRL_H

#include <stdint.h>

void solaris_manual_ctrl_set(int8_t throttle, int8_t steering);
void solaris_manual_ctrl_get(int8_t *throttle_out, int8_t *steering_out);

#endif // SOLARIS_MANUAL_CTRL_H
