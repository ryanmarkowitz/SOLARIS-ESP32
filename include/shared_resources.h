#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* Handles for shared semaphores / mutexes */
extern SemaphoreHandle_t actuator_mutex; // Only drive motors or panel motors are allowed to move, don't let them move at the same time

/* Handles for Queues and Task notification */
extern QueueHandle_t xModeQueue;
extern TaskHandle_t xSolarTracking;
extern TaskHandle_t xDriverFunction;