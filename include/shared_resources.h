#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* Handles for shared semaphores / mutexes */
extern SemaphoreHandle_t actuator_mutex; // Only drive motors or panel motors are allowed to move, don't let them move at the same time
extern SemaphoreHandle_t solaris_energy_monitor_resource;
extern SemaphoreHandle_t i2c_bus_mutex; // INA228 and ICM20948 share I2C_NUM_0 -- serializes every bus transaction between them
extern SemaphoreHandle_t pt_bus_mutex;  // solar_tracking and driver_function both call solaris_pt_read() on the same mux-select GPIOs + ADC channel

/* Handles for Queues and Task notification */
extern QueueHandle_t xEventQueue;
extern TaskHandle_t xSolarTracking;
extern TaskHandle_t xDriverFunction;
extern TaskHandle_t xMoveDecision;
extern TaskHandle_t xImuDrive;
extern TaskHandle_t xImuAlign;
extern TaskHandle_t xUltrasonic;
extern TaskHandle_t xImuCollision;

/*
 * Events sent through xEventQueue. driver_function's blocking waits react
 * to all three the instant one arrives: a BLE mode switch, an ultrasonic-
 * detected obstacle, or the IMU detecting SOLARIS has stalled/collided
 * while trying to drive. One combined queue (rather than a separate queue
 * per source) so driver_function never has to finish waiting on one queue
 * before it can see an event on another.
 */
typedef enum
{
    SOLARIS_EVENT_MODE_CHANGE,
    SOLARIS_EVENT_ULTRASONIC,
    SOLARIS_EVENT_IMU_COLLISION,
} solaris_event_type_t;

typedef struct
{
    solaris_event_type_t type;

    // valid when type == SOLARIS_EVENT_MODE_CHANGE. Plain uint8_t (rather
    // than solaris_mode_t) so this header doesn't have to include
    // solaris_mode.h -- cast to/from solaris_mode_t at the call site.
    uint8_t mode;

    // valid when type == SOLARIS_EVENT_ULTRASONIC. One event per
    // triggering sensor -- if two sensors trigger at once, ultrasonic_task
    // sends two separate events rather than combining them.
    uint8_t sensor_index; // which sensor triggered (0..SOLARIS_US_MAX_SENSORS-1)
    float distance_in;    // that sensor's distance reading

    // SOLARIS_EVENT_IMU_COLLISION uses no extra fields -- the event type
    // alone (stalled/collided) is all driver_function needs right now.
} solaris_event_t;
