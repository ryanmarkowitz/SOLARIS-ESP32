/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef SOLARIS_MODE_H
#define SOLARIS_MODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SOLARIS_MODE_STATIONARY = 0,
    SOLARIS_MODE_AUTOMATIC = 1,
    SOLARIS_MODE_MANUAL = 2,
} solaris_mode_t;

bool solaris_mode_set_from_u8(uint8_t raw_mode);
solaris_mode_t solaris_mode_get(void);
const char *solaris_mode_to_str(solaris_mode_t mode);

#endif // SOLARIS_MODE_H
