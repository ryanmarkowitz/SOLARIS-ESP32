#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <solar_tracking.h>

/* Handles for shared semaphores / mutexes */
static SemaphoreHandle_t actuator_mutex; // Only drive motors or panel motors are allowed to move, don't let them move at the same time

/* phototransistor values array */
solaris_pt_result_t[SOLARIS_PT_MAX_SENSORS];