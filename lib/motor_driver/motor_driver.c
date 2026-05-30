#include "motor_driver.h"
#include "esp_log.h"
#include <stdint.h>
#include <encoders.h>
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_gen.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define PWM_PERIOD_TICKS 50

#define TAG "MOTOR_DRIVER_SERVICE"

// Asssign pin outs for each motor
/*
MOTOR 0 - PAN MOTOR
MOTOR 1 - TILT MOTOR
MOTOR 2 - FRONT LEFT MOTOR
MOTOR 3 - FRONT RIGHT MOTOR
MOTOR 4 - REAR LEFT MOTOR
MOTOR 5 - REAR RIGHT MOTOR
*/
static const motor_pins_t motor_pins[NUM_MOTORS] = {
    {.pwm_gpio = 10, .dir_gpio = 11},
    {.pwm_gpio = 23, .dir_gpio = 32},
    {.pwm_gpio = 4, .dir_gpio = 7},
    {.pwm_gpio = 6, .dir_gpio = 9},
    {.pwm_gpio = 12, .dir_gpio = 7},
    {.pwm_gpio = 15, .dir_gpio = 9}};

static motor_t motors[NUM_MOTORS];

// The motors will share one timer, and each operator will be assigned to two motors
static mcpwm_timer_handle_t shared_timer = NULL;
static mcpwm_oper_handle_t shared_operators[3] = {NULL, NULL, NULL};

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

void motor_init(void)
{
    // Timer — defines frequency
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,         // timer ticks at 1MHz
        .period_ticks = PWM_PERIOD_TICKS, // resets every 50 ticks = 20kHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &shared_timer);

    // Three Operators tied to the shared timer
    for (int i = 0; i < 3; i++)
    {
        mcpwm_operator_config_t oper_config = {.group_id = 0};
        mcpwm_new_operator(&oper_config, &shared_operators[i]);
        mcpwm_operator_connect_timer(shared_operators[i], shared_timer);
    }

    // Each motor gets its own generator and its own comparator.
    // However an operator is tied to 2 motors
    // Motors 0-1 tied to operator 0
    // Motors 2-3 tied to operator 1
    // Motors 4-5 tied to operator 2
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        const motor_pins_t *pins = &motor_pins[i];
        mcpwm_comparator_config_t cmp_config = {.flags.update_cmp_on_tez = true};
        mcpwm_new_comparator(shared_operators[i / 2], &cmp_config, &motors[i].comparator); // global!

        mcpwm_generator_config_t gen_config = {.gen_gpio_num = pins->pwm_gpio};
        mcpwm_new_generator(shared_operators[i / 2], &gen_config, &motors[i].generator);

        // Behavior: high at timer start, low at comparator match
        mcpwm_generator_set_action_on_timer_event(motors[i].generator,
                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                               MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(motors[i].generator,
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                   motors[i].comparator, MCPWM_GEN_ACTION_LOW));

        gpio_set_direction(pins->dir_gpio, GPIO_MODE_INPUT_OUTPUT);

        // Set Duty cycle to 0% at init
        mcpwm_comparator_set_compare_value(motors[i].comparator, 0);
    }

    // Start — runs forever in hardware from here
    mcpwm_timer_enable(shared_timer);
    mcpwm_timer_start_stop(shared_timer, MCPWM_TIMER_START_NO_STOP);
}

// Set duty cycle to 0 for the motor to stop it
void stop_motor(int motor_id)
{
    mcpwm_comparator_set_compare_value(motors[motor_id].comparator, 0);
    vTaskDelay(pdMS_TO_TICKS(100)); // small delay before saving position to flash in case motor kept moving forward for some time
    save_position_to_flash();
}

// makes motor go forward at 25% duty cycle
void motor_go_forward(int motor_id)
{
    const motor_pins_t *pins = &motors[motor_id].pins;

    if (s_state != PANEL_AT_UPPER_LIMIT)
    {
        s_state = PANEL_OK;
        // set the motors direction to forward
        gpio_set_level(pins->dir_gpio, 1);

        mcpwm_comparator_set_compare_value(motors[motor_id].comparator, 5);
        ESP_LOGI(TAG, "Moving the motors forward");
    }
    else
    {
        ESP_LOGI(TAG, "Trying to move panel forward, but upper limit is reached");
    }
}

// makes motor go backward at 25% duty cycle
void motor_go_backward(int motor_id)
{
    const motor_pins_t *pins = &motors[motor_id].pins;

    if (s_state != PANEL_AT_LOWER_LIMIT)
    {
        s_state = PANEL_OK;
        // set the motors direction to reverse
        gpio_set_level(pins->dir_gpio, 0);

        mcpwm_comparator_set_compare_value(motors[motor_id].comparator, 5);
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
        int pulse_count;
        pulse_count = get_pulse_count();
        ESP_LOGI(TAG, "Current Pulse Position: %d", pulse_count);
        motor_go_forward(0);
        vTaskDelay(pdMS_TO_TICKS(10000));
        pulse_count = get_pulse_count();
        ESP_LOGI(TAG, "Current Pulse Position: %d", pulse_count);
        motor_go_forward(0);
        vTaskDelay(pdMS_TO_TICKS(10000));
        pulse_count = get_pulse_count();
        ESP_LOGI(TAG, "Current Pulse Position: %d", pulse_count);
        motor_go_backward(0);
        vTaskDelay(pdMS_TO_TICKS(10000));
        pulse_count = get_pulse_count();
        ESP_LOGI(TAG, "Current Pulse Position: %d", pulse_count);
        motor_go_backward(0);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}