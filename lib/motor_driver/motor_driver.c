#include "motor_driver.h"
#include "esp_log.h"
#include <stdint.h>
#include <encoders.h>
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define PWM_PIN 10
#define DIR_PIN 11
#define PWM_PERIOD_TICKS 50

#define TAG "MOTOR_DRIVER_SERVICE"

mcpwm_cmpr_handle_t motor_comparator = NULL;

// Track if panning of solar panel is at its maximum degree
static volatile panel_limit_state_t s_state = PANEL_OK;

panel_limit_state_t panel_get_limit_state(void)
{
    return s_state;
}

void panel_set_limit_state(panel_limit_state_t state)
{
    s_state = state;
}

void motor_init()
{
    // Timer — defines frequency
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,         // timer ticks at 1MHz
        .period_ticks = PWM_PERIOD_TICKS, // resets every 50 ticks = 20kHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &timer);

    // Operator — ties timer to outputs
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t oper_config = {.group_id = 0};
    mcpwm_new_operator(&oper_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    // Comparator — holds the duty cycle value
    mcpwm_comparator_config_t cmp_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(oper, &cmp_config, &motor_comparator); // global!

    // Generator — the actual GPIO output
    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t gen_config = {.gen_gpio_num = PWM_PIN};
    mcpwm_new_generator(oper, &gen_config, &generator);

    // Behavior: high at timer start, low at comparator match
    mcpwm_generator_set_action_on_timer_event(generator,
                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                           MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator,
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                               motor_comparator, MCPWM_GEN_ACTION_LOW));

    // Direction GPIO
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    // Set Duty cycle to 0% at init
    mcpwm_comparator_set_compare_value(motor_comparator, 0);

    // Start — runs forever in hardware from here
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}

// Set duty cycle to 0 for the motor to stop it
void stop_motor()
{
    mcpwm_comparator_set_compare_value(motor_comparator, 0);
    vTaskDelay(pdMS_TO_TICKS(100)); // small delay before saving position to flash in case motor kept moving forward for some time
    save_position_to_flash();
}

// makes motor go forward at 25% duty cycle
void motor_go_forward()
{
    if (s_state != PANEL_AT_UPPER_LIMIT)
    {
        s_state = PANEL_OK;
        // set the motors direction to forward
        gpio_set_level(DIR_PIN, 1);
        mcpwm_comparator_set_compare_value(motor_comparator, 13);
        ESP_LOGI(TAG, "Moving the motors forward");
    }
    else
    {
        ESP_LOGI(TAG, "Trying to move panel forward, but upper limit is reached");
    }
}

// makes motor go backward at 25% duty cycle
void motor_go_backward()
{
    if (s_state != PANEL_AT_LOWER_LIMIT)
    {
        s_state = PANEL_OK;
        // set the motors direction to reverse
        gpio_set_level(DIR_PIN, 0);
        mcpwm_comparator_set_compare_value(motor_comparator, 13);
        ESP_LOGI(TAG, "Moving the motors backward");
    }
    else
    {
        ESP_LOGI(TAG, "Trying to move panel backward but lower limit is reached");
    }
}

void test_motor()
{
    while (1)
    {
        motor_go_forward();
        vTaskDelay(pdMS_TO_TICKS(7500));
        motor_go_forward();
        vTaskDelay(pdMS_TO_TICKS(7500));
        motor_go_backward();
        vTaskDelay(pdMS_TO_TICKS(7500));
        motor_go_backward();
        vTaskDelay(pdMS_TO_TICKS(7500));
    }
}