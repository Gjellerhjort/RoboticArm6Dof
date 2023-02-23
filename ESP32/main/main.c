#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"

// custom components 
#include "stepper_a4988.h"
#include "servo.h"

#define LED_PIN 2
#define LED2_PIN 4
#define LED3_PIN 25

// definere servo pin
#define SERVO_PIN 25

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

#define LED_DELAY 1000
// MicroStepping
#define MS1 14
#define MS2 12
#define MS3 13


//#define LED_TIMER_GROUP TIMER_GROUP_0
//#define LED_TIMER_IDX TIMER_0
//#define LED_DELAY 500 // 1 ms


void led2_callback(void *arg)
{
    while (1) {
        stepper_moveMicrostep(1, 20000, 1, 1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(3000);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(2500);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(2000);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(1500);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(1400);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    char *ourTaskName = pcTaskGetName(NULL);
    ESP_LOGI(ourTaskName, "Hello Starting\n");
    
    gpio_reset_pin(LED_PIN);
    gpio_reset_pin(LED2_PIN);
    gpio_reset_pin(LED3_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);

    stepper_config(1, STEPPER_1_STEP, STEPPER_1_DIR);
    microstepping_config(MS1, MS2, MS3);
    stepper_Init();

    servo_init();
    
    void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution);


    xTaskCreatePinnedToCore(
        led2_callback, // function call name
        "blink_led2_task", // taksk name
        2048, // stack size
        NULL, // task paramerters
        5, // task proirity
        NULL, // task handler 
        1 // CPU core 
    );

    while(1) {
        servo_move(0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        servo_move(180);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}