#include <solaris_mode.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <solar_tracking.h>
#include "freertos/queue.h"
#include "shared_resources.h"
#include <motor_driver.h>
#include <quickselect.h>
#include <solaris_ina228.h>
#include <solaris_icm20948.h>
#include <solaris_ultrasonic.h>
#include <solaris_mode.h>
#include "driver.h"
#include <math.h>

#define TAG "Driving Function"

#define IMU_DRIVE_LOOP_MS 50
#define IMU_DRIVE_BASE_DUTY 0.75f
#define IMU_DRIVE_KP 0.01f
#define IMU_DRIVE_MAX_CORRECTION 0.25f

#define COMPASS_ALIGN_TOLERANCE_DEG 3.0f
#define RAD2DEG 57.29577951308232f

#define ULTRASONIC_POLL_MS 250
#define ULTRASONIC_TRIGGER_IN 12.0f

#define IMU_COLLISION_POLL_MS 250
// ICM-20948 default accel full-scale is +/-2g -> 16384 LSB per g (matches
// the gyro assumption: solaris_icm20948_init() never touches ACCEL_CONFIG).
#define IMU_ACCEL_SENS_LSB_PER_G 16384.0f
#define IMU_COLLISION_IMPACT_G 2.0f       // sudden jump in accel magnitude = jolt/impact
#define IMU_COLLISION_STALL_DELTA_G 0.02f // frame-to-frame change below this = "not moving"
#define IMU_COLLISION_STALL_SAMPLES 8     // consecutive quiet samples (~2s @ 250ms) while driving = stalled

/*
Framework for detecting if shade:
First make sure we aren't in sunrise / sunset window
Check energy monitor every second and store power it sees in a ring buffer
Every 3 minutes we will check the median of the past 30 seconds of info vs last 3 minutes. if there is a significant drop move to next checks
If cloud coverage percentage is low, move
Move in direction phototransistors on chassis recommends
Move in steps. we'll do 3 steps.
At each let energy monitor take 30 samples After settling panel. Then check if power gain matches or comes close to what the drop was
If we recieved power gain similar to the drop, stop movement
If we didn't recieve power gains, do another step.
If after 3 steps we don't recieve power gains, cut losses. Don't let sunk cost break the system.

*/

/*
0 1
2 3
*/

static volatile solaris_mode_t solaris_mode;

// xImuDrive/xImuAlign/xUltrasonic/xImuCollision/xMoveDecision are only
// assigned when their tasks are created in main.c. Several of those
// xTaskCreatePinnedToCore calls are currently commented out, leaving the
// handles NULL -- xTaskNotify/xTaskNotifyGive configASSERT on a NULL handle,
// which aborts the whole firmware. Guard every notify the same way
// gatt_svc.c already guards xTimeSynced.
static inline void notify_if_valid(TaskHandle_t task, uint32_t value, eNotifyAction action)
{
    if (task != NULL)
        xTaskNotify(task, value, action);
}

static inline void notify_give_if_valid(TaskHandle_t task)
{
    if (task != NULL)
        xTaskNotifyGive(task);
}

void driver_function(void *pvParameters)
{
    solaris_mode_set_from_u8(SOLARIS_MODE_MANUAL);
    solaris_mode = solaris_mode_get(); // initialize the mode
    int move_counter = 0;
    uint32_t bits;
    float median_power_last_3m;
    float median_power_last_30s;
    solaris_pt_result_t result[SOLARIS_PT_MAX_SENSORS / 2];
    int max = 0;
    int max_idx;
    int move_angle;
    solaris_event_t evt;
    while (1)
    {
        switch (solaris_mode)
        {
        case SOLARIS_MODE_AUTOMATIC:
            // // If we recieve a signal we should move
            if (xTaskNotifyWait(0x00, ULONG_MAX, &bits, 0) == pdTRUE)
            {
                move_counter = 0;
                // get what the median of the previous 3 minutes was to compare after moving
                memcpy(&median_power_last_3m, &bits, sizeof(median_power_last_3m));
            start:
                if (move_counter > 3)
                {
                    solaris_mode_set_from_u8(0);
                    solaris_mode = solaris_mode_get();
                    goto done;
                }
                // Find direction to move in
                solaris_pt_read(pt, result, false);
                for (int i = 0; i < SOLARIS_PT_MAX_SENSORS / 2; i++)
                {
                    if (max < result[i].mv)
                    {
                        max = result[i].mv;
                        max_idx = i;
                    }
                }

                // figure out from phototransistor array which direction to turn
                if (max_idx == 0)
                {
                    move_angle = 45;
                }
                else if (max_idx == 1)
                {
                    move_angle = -45;
                }
                else if (max_idx == 2)
                {
                    move_angle = 135;
                }
                else
                {
                    move_angle = -135;
                }

                // turn SOLARIS to the highest light level phototransistor
                xSemaphoreTake(actuator_mutex, portMAX_DELAY);
                motor_turn_degrees(imu_handle, move_angle);
                xSemaphoreGive(actuator_mutex);

                // Drive straight for 3 seconds in the direction max_idx picked.
                // imu_drive_task corrects for yaw drift using the gyro, and owns
                // actuator_mutex for the duration of the drive.
                notify_if_valid(xImuDrive, max_idx < 2 ? IMU_DRIVE_FORWARD : IMU_DRIVE_BACKWARD, eSetValueWithOverwrite);
                notify_if_valid(xUltrasonic, ULTRASONIC_TASK_START, eSetValueWithOverwrite);
                notify_if_valid(xImuCollision, IMU_COLLISION_TASK_START, eSetValueWithOverwrite);

                // Wait up to 3 seconds while driving. xQueueReceive wakes up the
                // instant anything arrives, so a mode switch, ultrasonic obstacle,
                // or IMU collision all get handled immediately, not just at the 3s mark.
                if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(3000)) == pdPASS)
                {
                    notify_if_valid(xImuDrive, IMU_DRIVE_STOP, eSetValueWithOverwrite);
                    notify_if_valid(xUltrasonic, ULTRASONIC_TASK_STOP, eSetValueWithOverwrite);
                    notify_if_valid(xImuCollision, IMU_COLLISION_TASK_STOP, eSetValueWithOverwrite);

                    if (evt.type == SOLARIS_EVENT_MODE_CHANGE)
                    {
                        solaris_mode = (solaris_mode_t)evt.mode;
                        goto done;
                    }

                    // Ultrasonic obstacle or IMU collision: back up, turn away,
                    // and re-evaluate direction from the top.
                    if (evt.type == SOLARIS_EVENT_ULTRASONIC)
                    {
                        // wait 5 seconds before trying to move around the object.
                        if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(5000)) == pdPASS)
                        {
                            if (evt.type == SOLARIS_EVENT_MODE_CHANGE)
                            {
                                solaris_mode = (solaris_mode_t)evt.mode;
                                goto done;
                            }
                        }

                        notify_if_valid(xUltrasonic, ULTRASONIC_TASK_START, eSetValueWithOverwrite);
                        if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(500)) == pdPASS)
                        {
                            notify_if_valid(xUltrasonic, ULTRASONIC_TASK_STOP, eSetValueWithOverwrite);
                            if (evt.type == SOLARIS_EVENT_MODE_CHANGE)
                            {
                                solaris_mode = (solaris_mode_t)evt.mode;
                                goto done;
                            }
                            // object is still detected
                            else if (evt.type == SOLARIS_EVENT_ULTRASONIC)
                            {
                                // back up and turn in direction of best sunlight again
                                notify_if_valid(xImuDrive, max_idx < 2 ? IMU_DRIVE_FORWARD : IMU_DRIVE_BACKWARD, eSetValueWithOverwrite);
                                vTaskDelay(pdMS_TO_TICKS(1000));
                                notify_if_valid(xImuDrive, IMU_DRIVE_STOP, eSetValueWithOverwrite);
                                move_counter++;
                                goto start;
                            }
                        }

                        ESP_LOGW(TAG, "ultrasonic obstacle: sensor=%d distance=%.1fin", evt.sensor_index, evt.distance_in);
                    }
                    else // SOLARIS_EVENT_IMU_COLLISION
                    {
                        notify_if_valid(xImuDrive, max_idx < 2 ? IMU_DRIVE_FORWARD : IMU_DRIVE_BACKWARD, eSetValueWithOverwrite);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        notify_if_valid(xImuDrive, IMU_DRIVE_STOP, eSetValueWithOverwrite);
                        move_counter++;
                        goto start;
                        ESP_LOGW(TAG, "imu collision/stall detected");
                    }
                }

                // Stop moving the motors
                notify_if_valid(xImuDrive, IMU_DRIVE_STOP, eSetValueWithOverwrite);
                notify_if_valid(xUltrasonic, ULTRASONIC_TASK_STOP, eSetValueWithOverwrite);
                notify_if_valid(xImuCollision, IMU_COLLISION_TASK_STOP, eSetValueWithOverwrite);

                // Reorient to face whichever of north/south is the shorter turn.
                notify_give_if_valid(xImuAlign);

                // increment move_counter
                move_counter++;

                // After reorientation, allow the panel to self adjust
                xTaskNotifyGive(xSolarTracking);

                // wait up to 30 seconds for the panel/power to settle. If we recieve mode change in this process, just change mode
                if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(30000)) == pdPASS)
                {
                    if (evt.type == SOLARIS_EVENT_MODE_CHANGE)
                    {
                        solaris_mode = (solaris_mode_t)evt.mode;
                        goto done;
                    }
                }

                // Otherwise Check the new 30 second power gain compared to old previous 3m. If we made up the power stay put
                // solaris_power_buffer/solaris_power_buffer_idx are written by solaris_ina228_1s_read under this
                // same mutex, and solaris_windowed_median's internal scratch buffer isn't safe to enter from two
                // tasks at once -- both reasons this must be held here too.
                xSemaphoreTake(solaris_energy_monitor_resource, portMAX_DELAY);
                median_power_last_30s = solaris_windowed_median(solaris_power_buffer, SOLARIS_RING_BUFFER_SIZE, solaris_power_buffer_idx, 30);
                xSemaphoreGive(solaris_energy_monitor_resource);
                if (median_power_last_30s * 1.05 < median_power_last_3m)
                {
                    // We couldn't make up the power loss. If we haven't done 3 steps yet, try moving again
                    if (move_counter < 3)
                    {
                        goto start;
                    }
                    // We've moved 3 times already. Cut losses and move to stationary mode
                    else
                    {
                        // set the mode to stationary
                        solaris_mode_set_from_u8(0);
                        solaris_mode = solaris_mode_get();
                        goto done;
                    }
                }
            }

            // We didn't move, make sure solar panel doesn't need to reorient itself
            else
            {
                xTaskNotifyGive(xSolarTracking);
            }
            break;
        case SOLARIS_MODE_MANUAL:;
            break;
        case SOLARIS_MODE_STATIONARY:
            xTaskNotifyGive(xSolarTracking);
            break;
        }
    // Generic poll, runs regardless of mode. Only mode switches are acted on
    // here -- ultrasonic/IMU events are just logged, since we don't know if
    // we're actively driving right now (e.g. this also runs in manual mode).
    done:
        if (xQueueReceive(xEventQueue, &evt, pdMS_TO_TICKS(250)) == pdPASS)
        {
            if (evt.type == SOLARIS_EVENT_MODE_CHANGE)
            {
                solaris_mode = (solaris_mode_t)evt.mode;
                ESP_LOGI(TAG, "mode changed %s", solaris_mode_to_str(solaris_mode));
            }
        }
    }
}

void imu_drive_task(void *pvParameters)
{
    solaris_icm20948_result_t sample;
    uint32_t cmd;

    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, &cmd, portMAX_DELAY);
        if (cmd != IMU_DRIVE_FORWARD && cmd != IMU_DRIVE_BACKWARD)
            continue;

        bool forward = (cmd == IMU_DRIVE_FORWARD);

        xSemaphoreTake(actuator_mutex, portMAX_DELAY);

        float yaw_error_deg = 0.0f;
        TickType_t last_tick = xTaskGetTickCount();

        while (1)
        {
            uint32_t stop_cmd;
            if (xTaskNotifyWait(0x00, ULONG_MAX, &stop_cmd, 0) == pdTRUE && stop_cmd == IMU_DRIVE_STOP)
                break;

            TickType_t now_tick = xTaskGetTickCount();
            float dt_s = (float)(now_tick - last_tick) * portTICK_PERIOD_MS / 1000.0f;
            last_tick = now_tick;

            if (solaris_icm20948_read(imu_handle, &sample) == ESP_OK)
            {
                float gyro_dps = (float)sample.gyro_z / IMU_GYRO_SENS_LSB_PER_DPS;
                yaw_error_deg += gyro_dps * dt_s;
            }

            // Slow the side we've drifted toward so we straighten back out.
            // Sign is a starting assumption for which way gyro_z drift maps
            // to left/right — flip if field testing shows it over-corrects.
            float correction = yaw_error_deg * IMU_DRIVE_KP;
            if (correction > IMU_DRIVE_MAX_CORRECTION)
                correction = IMU_DRIVE_MAX_CORRECTION;
            if (correction < -IMU_DRIVE_MAX_CORRECTION)
                correction = -IMU_DRIVE_MAX_CORRECTION;

            float left_duty = IMU_DRIVE_BASE_DUTY - correction;
            float right_duty = IMU_DRIVE_BASE_DUTY + correction;
            if (left_duty < 0.0f)
                left_duty = 0.0f;
            if (left_duty > 1.0f)
                left_duty = 1.0f;
            if (right_duty < 0.0f)
                right_duty = 0.0f;
            if (right_duty > 1.0f)
                right_duty = 1.0f;

            if (forward)
            {
                motor_go_forward(MOTOR_LEFT_ID, left_duty);
                motor_go_backward(MOTOR_RIGHT_ID, right_duty);
            }
            else
            {
                motor_go_backward(MOTOR_LEFT_ID, left_duty);
                motor_go_forward(MOTOR_RIGHT_ID, right_duty);
            }

            vTaskDelay(pdMS_TO_TICKS(IMU_DRIVE_LOOP_MS));
        }

        stop_motor(MOTOR_LEFT_ID);
        stop_motor(MOTOR_RIGHT_ID);
        xSemaphoreGive(actuator_mutex);
    }
}

static float normalize_deg(float deg)
{
    while (deg < 0.0f)
        deg += 360.0f;
    while (deg >= 360.0f)
        deg -= 360.0f;
    return deg;
}

void imu_align_task(void *pvParameters)
{
    solaris_icm20948_result_t sample;

    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);

        xSemaphoreTake(actuator_mutex, portMAX_DELAY);

        if (solaris_icm20948_read(imu_handle, &sample) == ESP_OK)
        {
            // Tilt-uncompensated compass heading: 0 = north, increasing
            // clockwise (matches motor_turn_degrees' positive = right
            // convention). Assumes the IMU is mounted flat with mag_x/mag_y
            // aligned to chassis forward/right — verify against a real
            // compass and swap/negate the atan2 arguments if it's off.
            float heading_deg = normalize_deg(atan2f((float)sample.mag_y, (float)sample.mag_x) * RAD2DEG);

            // Face whichever of north (0) or south (180) is angularly closer
            float dist_to_north = fminf(fabsf(heading_deg), 360.0f - fabsf(heading_deg));
            float dist_to_south = fminf(fabsf(heading_deg - 180.0f), 360.0f - fabsf(heading_deg - 180.0f));
            float target_deg = (dist_to_north <= dist_to_south) ? 0.0f : 180.0f;

            // Shortest signed turn to reach the target, in (-180, 180]:
            // positive turns right, negative turns left — always the
            // faster of the two directions by construction.
            float delta_deg = target_deg - heading_deg;
            while (delta_deg > 180.0f)
                delta_deg -= 360.0f;
            while (delta_deg <= -180.0f)
                delta_deg += 360.0f;

            if (fabsf(delta_deg) > COMPASS_ALIGN_TOLERANCE_DEG)
            {
                motor_turn_degrees(imu_handle, delta_deg);
            }
        }
        else
        {
            ESP_LOGW(TAG, "imu_align_task: magnetometer read failed, skipping alignment");
        }

        xSemaphoreGive(actuator_mutex);
    }
}

void ultrasonic_task(void *pvParameters)
{
    solaris_us_result_t results[SOLARIS_US_MAX_SENSORS];
    uint32_t cmd;

    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, &cmd, portMAX_DELAY);
        if (cmd != ULTRASONIC_TASK_START)
            continue;

        while (1)
        {
            uint32_t stop_cmd;
            if (xTaskNotifyWait(0x00, ULONG_MAX, &stop_cmd, 0) == pdTRUE && stop_cmd == ULTRASONIC_TASK_STOP)
                break;

            if (solaris_us_read(us_handle, results) == ESP_OK)
            {
                int dir = gpio_get_level(42);
                if (dir == 1)
                // TODO MAP THE ULTRASONIC INDEXES CORRECTLY
                { // motors are driving forward. Only check front two ultrasonics
                    if (results[0].inches <= ULTRASONIC_TRIGGER_IN || results[1].inches <= ULTRASONIC_TRIGGER_IN)
                    {
                        int idx = (results[0].inches < results[1].inches) ? 0 : 1;
                        solaris_event_t evt = {
                            .type = SOLARIS_EVENT_ULTRASONIC,
                            .sensor_index = (uint8_t)idx,
                            .distance_in = results[idx].inches,
                        };
                        xQueueSend(xEventQueue, &evt, 0);
                    }
                }
                else // motors are driving backward. Only check back two ultrasonics
                {
                    if (results[2].inches <= ULTRASONIC_TRIGGER_IN || results[3].inches <= ULTRASONIC_TRIGGER_IN)
                    {
                        int idx = (results[2].inches < results[3].inches) ? 2 : 3;
                        solaris_event_t evt = {
                            .type = SOLARIS_EVENT_ULTRASONIC,
                            .sensor_index = (uint8_t)idx,
                            .distance_in = results[idx].inches,
                        };
                        xQueueSend(xEventQueue, &evt, 0);
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_POLL_MS));
        }
    }
}

void imu_collision_task(void *pvParameters)
{
    solaris_icm20948_result_t sample;
    uint32_t cmd;

    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, &cmd, portMAX_DELAY);
        if (cmd != IMU_COLLISION_TASK_START)
            continue;

        float last_accel_mag_g = 1.0f; // assume starting at rest (~1g of gravity)
        int stall_quiet_samples = 0;

        while (1)
        {
            uint32_t stop_cmd;
            if (xTaskNotifyWait(0x00, ULONG_MAX, &stop_cmd, 0) == pdTRUE && stop_cmd == IMU_COLLISION_TASK_STOP)
                break;

            if (solaris_icm20948_read(imu_handle, &sample) == ESP_OK)
            {
                float accel_mag_g = sqrtf((float)sample.accel_x * sample.accel_x +
                                          (float)sample.accel_y * sample.accel_y +
                                          (float)sample.accel_z * sample.accel_z) /
                                    IMU_ACCEL_SENS_LSB_PER_G;

                if (accel_mag_g > IMU_COLLISION_IMPACT_G)
                {
                    // Sudden jolt -- report it immediately.
                    solaris_event_t evt = {.type = SOLARIS_EVENT_IMU_COLLISION};
                    xQueueSend(xEventQueue, &evt, 0);
                    stall_quiet_samples = 0;
                }
                else if (fabsf(accel_mag_g - last_accel_mag_g) < IMU_COLLISION_STALL_DELTA_G)
                {
                    // Barely any change in accel magnitude while motors are
                    // commanded on -- SOLARIS likely isn't actually moving.
                    stall_quiet_samples++;
                    if (stall_quiet_samples >= IMU_COLLISION_STALL_SAMPLES)
                    {
                        solaris_event_t evt = {.type = SOLARIS_EVENT_IMU_COLLISION};
                        xQueueSend(xEventQueue, &evt, 0);
                        stall_quiet_samples = 0; // don't spam the queue every loop after this
                    }
                }
                else
                {
                    stall_quiet_samples = 0;
                }
                last_accel_mag_g = accel_mag_g;
            }

            vTaskDelay(pdMS_TO_TICKS(IMU_COLLISION_POLL_MS));
        }
    }
}
