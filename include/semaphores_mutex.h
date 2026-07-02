#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Handles for shared semaphores / mutexes */
static SemaphoreHandle_t actuator_mutex; // Only drive motors or panel motors are allowed to move, don't let them move at the same time