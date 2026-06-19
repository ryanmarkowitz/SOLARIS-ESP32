#include <solaris_mode.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static volatile solaris_mode_t solaris_mode;

void driver_function()
{
    solaris_mode = solaris_mode_get; // initialize the mode
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
        case SOLARIS_MODE_MANUAL:
            /*
            Await commands from phone. Move according to those commands
            */
            break;
        case SOLARIS_MODE_STATIONARY:
            /*
            Move panel if needed
            */
            break;
        }
        // xQueueRecieve with timeout. Either get instructions or run automatic / stationary again
    }
}

void write_logs_and_buffers()
{
    // TODO make a task that runs every 30 seconds to capture log information and store information in buffers
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}