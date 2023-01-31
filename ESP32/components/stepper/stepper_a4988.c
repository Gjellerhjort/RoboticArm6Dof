#include <stdio.h>
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
volatile uint16_t stepper_1_count = 0; // dette er hvor mange steps stepper 1 skal udfører
volatile uint8_t stepper_1_direction = 1; // dette er retning motoren skal bevæge sig
volatile uint16_t stepper_2_count = 0;
volatile uint8_t stepper_2_direction = 1;
volatile uint16_t stepper_3_count = 0;
volatile uint8_t stepper_3_direction = 1;
volatile uint16_t stepper_4_count = 0;
volatile uint8_t stepper_4_direction = 1;
volatile uint16_t stepper_5_count = 0;
volatile uint8_t stepper_5_direction = 1;
volatile uint16_t stepper_6_count = 0;
volatile uint8_t stepper_6_direction = 1;

void call_stepper()
{
    ESP_LOGI("Stepper Driver" ,"Hello from Stepper driver");
}

static void stepper_timer_callback(void* arg)
{
    // motor 1A
    if(stepper_1_count > 0){
        gpio_set_level(STEPPER_1_DIR, stepper_1_direction); // sets DIR arro
        // makes step 
        gpio_set_level(STEPPER_1_STEP, 1); // sets STEP pin HIGH
        gpio_set_level(STEPPER_1_STEP, 0); // sets STEP pin LOW
        stepper_1_count--;
    } else
    {
        gpio_set_level(STEPPER_1_STEP, 0);
    }

    // motor 2A

    if(stepper_2_count > 0){
        gpio_set_level(STEPPER_2_DIR, stepper_2_direction);
        gpio_set_level(STEPPER_2_STEP, 1); //generate step
        gpio_set_level(STEPPER_2_STEP, 0);
        stepper_2_count--;
    } else
    {
        gpio_set_level(STEPPER_2_STEP, 0);
    }

    // motor 3A

    if(stepper_3_count > 0){
        gpio_set_level(STEPPER_3_DIR, stepper_3_direction);
        gpio_set_level(STEPPER_3_STEP, 1); //generate step
        gpio_set_level(STEPPER_3_STEP, 0);
        stepper_3_count--;
    } else
    {
        gpio_set_level(STEPPER_3_STEP, 0);
    }

    // motor 4A

    if(stepper_4_count > 0){
        gpio_set_level(STEPPER_4_DIR, stepper_4_direction);
        gpio_set_level(STEPPER_4_STEP, 1); //generate step
        gpio_set_level(STEPPER_4_STEP, 0);
        stepper_4_count--;
    } else
    {
        gpio_set_level(STEPPER_4_STEP, 0);
    }

    // motor 5C
    if(stepper_5_count > 0){
        gpio_set_level(STEPPER_5_DIR, stepper_5_direction);
        gpio_set_level(STEPPER_5_STEP, 1); //generate step
        gpio_set_level(STEPPER_5_STEP, 0);
        stepper_5_count--;
    } else
    {
        gpio_set_level(STEPPER_5_STEP, 0);
    }

    // motor 6C

    if(stepper_6_count > 0){
        gpio_set_level(STEPPER_6_DIR, stepper_6_direction);
        gpio_set_level(STEPPER_6_STEP, 1); //generate step
        gpio_set_level(STEPPER_6_STEP, 0);
        stepper_6_count--;
    } else
    {
        gpio_set_level(STEPPER_6_STEP, 0);
    }
}

void stepper_move2000(uint8_t direction)
    {
        stepper_1_count = 2000;
        stepper_1_direction = direction;
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
    if (motor_num == 1) // checker om det er motor 1A
    { 
        stepper_1_count = steps; // sætter antal steps som motoren skal bevæge sig  
        stepper_1_direction = direction; // sætter retning motoren skal bevæge sig
    } else if (motor_num == 2)  // motor 2A 200
    { 
        stepper_2_count = steps;   
        stepper_2_direction = direction;
    } else if (motor_num == 3) // motor 3A 200
    {
        stepper_3_count = steps;   
        stepper_3_direction = direction;
    } else if (motor_num == 4) // motor 4A 200
    {
        stepper_4_count = steps;  
        stepper_4_direction = direction;
    } else if (motor_num == 5) // motor 5C 200
    {
        stepper_5_count = steps;  
        stepper_5_direction = direction;
    } else if (motor_num == 6) // motor 6C 200
    {
        stepper_6_count = steps;  
        stepper_6_direction = direction;
    }
}

void stepper_Init()
{
    gpio_config_t stepper_gpio_conf;

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step pins
    stepper_gpio_conf.pin_bit_mask = 
      (1ULL<<STEPPER_1_DIR) | (1ULL<<STEPPER_4_DIR) 
    | (1ULL<<STEPPER_2_DIR) | (1ULL<<STEPPER_5_DIR)
    | (1ULL<<STEPPER_3_DIR) | (1ULL<<STEPPER_6_DIR)
    |  (1ULL<<STEPPER_1_STEP) | (1ULL<<STEPPER_4_STEP) 
    | (1ULL<<STEPPER_2_STEP) | (1ULL<<STEPPER_5_STEP)
    | (1ULL<<STEPPER_3_STEP) | (1ULL<<STEPPER_6_STEP);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);

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
    ESP_LOGI("Stepper Driver" ,"All Pins Init");
}
