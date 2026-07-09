#ifndef SOLARIS_DRIVER
#define SOLARIS_DRIVER

// Commands sent to imu_drive_task via xTaskNotify (eSetValueWithOverwrite).
typedef enum
{
    IMU_DRIVE_STOP = 0,
    IMU_DRIVE_FORWARD = 1,
    IMU_DRIVE_BACKWARD = 2,
} imu_drive_cmd_t;

// Commands sent to ultrasonic_task via xTaskNotify (eSetValueWithOverwrite).
typedef enum
{
    ULTRASONIC_TASK_STOP = 0,
    ULTRASONIC_TASK_START = 1,
} ultrasonic_task_cmd_t;

// Commands sent to imu_collision_task via xTaskNotify (eSetValueWithOverwrite).
typedef enum
{
    IMU_COLLISION_TASK_STOP = 0,
    IMU_COLLISION_TASK_START = 1,
} imu_collision_task_cmd_t;

void driver_function(void *pvParameters);
void write_log_and_buffers();

/*
 * Persistent task: waits for an IMU_DRIVE_FORWARD/IMU_DRIVE_BACKWARD
 * notification, then drives straight in that direction, using the IMU
 * gyro to correct for yaw drift, until it receives an IMU_DRIVE_STOP
 * notification. Owns actuator_mutex for the duration of the drive (taken
 * once when driving starts, given back once when it stops).
 */
void imu_drive_task(void *pvParameters);

/*
 * Persistent task: waits for any notification (value ignored), then reads
 * the magnetometer, picks whichever of north (0 deg) or south (180 deg) is
 * angularly closer, and turns SOLARIS the shorter way (right or left,
 * whichever is faster) to face it via motor_turn_degrees(). Owns
 * actuator_mutex for the duration of the turn.
 */
void imu_align_task(void *pvParameters);

/*
 * Persistent task: waits for an ULTRASONIC_TASK_START notification, then
 * every 250ms reads all ultrasonic sensors. Any sensor reading within
 * ULTRASONIC_TRIGGER_IN inches sends its own SOLARIS_EVENT_ULTRASONIC event
 * on xEventQueue (sensor index + distance) -- multiple sensors triggering
 * at once means multiple events, not one combined event. Stops polling on
 * an ULTRASONIC_TASK_STOP notification.
 */
void ultrasonic_task(void *pvParameters);

/*
 * Persistent task: waits for an IMU_COLLISION_TASK_START notification, then
 * every 250ms reads the accelerometer and sends a SOLARIS_EVENT_IMU_COLLISION
 * event on xEventQueue if either: (a) accel magnitude spikes above
 * IMU_COLLISION_IMPACT_G (a jolt/impact), or (b) accel magnitude barely
 * changes for IMU_COLLISION_STALL_SAMPLES consecutive samples while motors
 * are commanded on (stuck against something). Stops polling on an
 * IMU_COLLISION_TASK_STOP notification.
 */
void imu_collision_task(void *pvParameters);

#endif
