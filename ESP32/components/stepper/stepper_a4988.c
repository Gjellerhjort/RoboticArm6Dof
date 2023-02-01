#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>
#include <driver/gpio.h>

// definere Stepper Step pin
#define STEPPER_1_STEP 27 //defining STEP pin of first motor
#define STEPPER_2_STEP 19    
#define STEPPER_3_STEP 5
#define STEPPER_4_STEP 16
#define STEPPER_5_STEP 0
#define STEPPER_6_STEP 15

// definere Stepper DIR pin
#define STEPPER_1_DIR 26 //defining DIR pin of first motor
#define STEPPER_2_DIR 23
#define STEPPER_3_DIR 18
#define STEPPER_4_DIR 17
#define STEPPER_5_DIR 4
#define STEPPER_6_DIR 2

// Timer
esp_timer_handle_t timer_handle;
volatile uint16_t INTERVAL_US = 2000;

// definere stepper variables
volatile uint8_t NUM_STEPPERS = 0;

typedef struct StepperInfo {
    uint8_t step_pin;
    uint8_t dir_pin;
    volatile uint16_t step_count;
    volatile uint8_t step_direction;
    volatile uint16_t min_interval;
} StepperInfo;



void call_stepper()
{
    ESP_LOGI("Stepper Driver" ,"Hello from Stepper driver");
}

static void stepper_timer_callback(void* arg)
{
    volatile StepperInfo *steppers = &steppers[NUM_STEPPERS];
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if(steppers[i].step_count > 0){
            gpio_set_level(steppers[i].dir_pin, steppers[i].step_direction); // sets DIR arro
            // makes step 
            gpio_set_level(steppers[i].step_pin, 1); // sets STEP pin HIGH
            gpio_set_level(steppers[i].step_pin, 0); // sets STEP pin LOW
            steppers[i].step_count--;
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

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    ESP_LOGI("stepper driver", "steps runing");
    // motor 1A steps
    steppers[motor_num-1].step_count = steps;
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
    steppers[i].min_interval = 2;
    
    gpio_config_t stepper_gpio_conf;

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<steppers[i].step_pin) | (1ULL<<steppers[i].dir_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
    NUM_STEPPERS++;
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
    stepper_config(1, STEPPER_1_STEP, STEPPER_1_DIR);
    stepper_config(2, STEPPER_2_STEP, STEPPER_2_DIR);
    stepper_config(3, STEPPER_3_STEP, STEPPER_3_DIR);
    stepper_config(4, STEPPER_4_STEP, STEPPER_4_DIR);
    stepper_config(5, STEPPER_5_STEP, STEPPER_5_DIR);
    stepper_config(6, STEPPER_6_STEP, STEPPER_6_DIR);

    volatile StepperInfo steppers[NUM_STEPPERS];

    stepper_start_timer();
    ESP_LOGI("Stepper Driver" ,"All Pins Init");
}