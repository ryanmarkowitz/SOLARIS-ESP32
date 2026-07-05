#include <solaris_mode.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <solar_tracking.h>
#include "freertos/queue.h"
#include "shared_resources.h"

#define TAG "Driving Function"

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

static volatile solaris_mode_t solaris_mode;

void driver_function(void *pvParameters)
{
    solaris_mode = solaris_mode_get(); // initialize the mode
    while (1)
    {
        switch (solaris_mode)
        {
        case SOLARIS_MODE_AUTOMATIC:
            /*
            1. check if in shade
                a. if no shade detected move panel if necessary and skip to last step
                b. if shade is detected start new moving process step 2
            2. Determine which direction SOLARIS should move
            3. Rotate the robot to match that direction
            4. Move the robot in that direction for a period of time.
            5. Check if there was an expected power gain.
                a. If expected power gain is achieved skip stop moving robot
                b. If expected power gain is not achieved try moving forward again
            */
            break;
        case SOLARIS_MODE_MANUAL:;
            break;
        case SOLARIS_MODE_STATIONARY:
            xTaskNotifyGive(xSolarTracking);
            break;
        }
        if (xQueueReceive(xModeQueue, &solaris_mode, pdMS_TO_TICKS(250)) == pdPASS)
        {
            ESP_LOGI(TAG, "mode changed %s", solaris_mode_to_str(solaris_mode));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
