#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>
#include <driver/gpio.h>

// Timer
esp_timer_handle_t timer_handle;
volatile uint16_t INTERVAL_US = 2000;

// definere stepper variables
gpio_config_t stepper_gpio_conf;
volatile uint8_t NUM_STEPPERS = 0;

typedef struct StepperInfo {
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t ms1_pin;
    uint8_t ms2_pin;
    uint8_t ms3_pin;

    volatile uint8_t step_res; // step size resolution (microstepping)
    volatile uint8_t total_steps;
    volatile uint16_t step_count;
    volatile uint8_t step_direction;
    volatile uint16_t min_interval;
} StepperInfo;

volatile StepperInfo steppers[1];

void call_stepper()
{
    ESP_LOGI("Stepper Driver" ,"Hello from Stepper driver");
}

void stepper_setMircostepping(uint8_t motor_num ,uint8_t res){
    switch (res)
    {
        case 2: // Half step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 0);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 4: // 1/4 step
            gpio_set_level(steppers[motor_num].ms1_pin, 0);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 8: // 1/8 step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 16: // 1/16 step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 1);
            break;

        default:
            gpio_set_level(steppers[motor_num].ms1_pin, 0);
            gpio_set_level(steppers[motor_num].ms2_pin, 0);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);

    }
}

static void stepper_timer_callback(void* arg)
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if(steppers[i].step_count < steppers[i].total_steps){
            gpio_set_level(steppers[i].dir_pin, steppers[i].step_direction); // sets DIR arro
            if (steppers[i].step_res != 0)
            {
                stepper_setMircostepping(i, steppers[i].step_res);
                steppers[i].step_res = 0;

            }
            // makes step 
            gpio_set_level(steppers[i].step_pin, 1); // sets STEP pin HIGH
            gpio_set_level(steppers[i].step_pin, 0); // sets STEP pin LOW
            steppers[i].step_count++;
        } else {
            gpio_set_level(steppers[i].step_pin, 0);
        }
    }
}

void stepper_setSpeed(uint16_t new_timer_period) 
{

    INTERVAL_US = new_timer_period;
    // Restarts the timer with new timer interval
    esp_timer_restart(timer_handle, INTERVAL_US);
}   

void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution) {
    ESP_LOGI("stepper driver", "steps runing");
    // motor 1A steps
    steppers[motor_num-1].total_steps = steps;
    steppers[motor_num-1].step_count = 0;
    steppers[motor_num-1].step_direction = direction;
    steppers[motor_num-1].step_res = stepping_resolution;
}

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    ESP_LOGI("stepper driver", "steps runing");
    // motor 1A steps
    steppers[motor_num-1].total_steps = steps;
    steppers[motor_num-1].step_direction = direction;
}

void stepper_config_reset()
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].step_pin = 0;
        steppers[i].dir_pin = 0;
        steppers[i].step_count = 0;
        steppers[i].step_direction = 0;
        steppers[i].min_interval = 0;   
    }
}

void stepper_config(uint8_t step_num, uint8_t step_pin, uint8_t dir_pin)
{
    uint8_t i = step_num-1;
    steppers[i].step_pin = step_pin;
    steppers[i].dir_pin = dir_pin;
    steppers[i].step_count = 0;
    steppers[i].step_direction = 1;
    steppers[i].total_steps = 0;
    steppers[i].min_interval = 1500;

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<steppers[i].step_pin) | (1ULL<<steppers[i].dir_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
    NUM_STEPPERS++;
}

void microstepping_config(uint8_t ms1_pin, uint8_t ms2_pin, uint8_t ms3_pin)
{
    for(int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].ms1_pin = ms1_pin;
        steppers[i].ms2_pin = ms2_pin;
        steppers[i].ms3_pin = ms3_pin;   
    }

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<ms1_pin) | (1ULL<<ms2_pin) | (1ULL<<ms3_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
}

void stepper_start_timer()
{
    // Create the timer
    esp_timer_create_args_t timer_conf = {
        .callback = &stepper_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stepper_timer"
    };

    esp_timer_create(&timer_conf, &timer_handle);

    // Start the timer
    esp_timer_start_periodic(timer_handle, INTERVAL_US);
}

void stepper_Init()
{

    stepper_start_timer();
    ESP_LOGI("Stepper Driver" ,"All Pins Init");
}