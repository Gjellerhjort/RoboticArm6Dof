#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"

// custom components 
#include "stepper_a4988.h"

#define LED_PIN 2
#define LED2_PIN 4
#define LED3_PIN 25

#define LED_TIMER_GROUP TIMER_GROUP_0
#define LED_TIMER_IDX TIMER_0
#define LED_DELAY 500 // 1 ms


void led2_callback(void *arg)
{
    while (1) {
        ESP_LOGI("Motor Speed", "Core Id: %d\n", esp_cpu_get_core_id());
        stepper_setSpeed(2500);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_setSpeed(1200);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void led3_callback(void *arg)
{
    while (1) {
        ESP_LOGI("led3_loop", "Core Id: %d\n", esp_cpu_get_core_id());
        gpio_set_level(LED3_PIN, 1);
        vTaskDelay(LED_DELAY / portTICK_PERIOD_MS*4);
        gpio_set_level(LED3_PIN, 0);
        vTaskDelay(LED_DELAY / portTICK_PERIOD_MS*4);
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

    stepper_Init();

    xTaskCreatePinnedToCore(
        led2_callback, // function call name
        "blink_led2_task", // taksk name
        2048, // stack size
        NULL, // task paramerters
        5, // task proirity
        NULL, // task handler 
        1 // CPU core 
    );

    xTaskCreatePinnedToCore(
        led3_callback, // function call name
        "blink_led3_task", // taksk name
        2048, // stack size
        NULL, // task paramerters
        5, // task proirity
        NULL, // task handler 
        tskNO_AFFINITY // CPU core
    );

    while(1) {
        stepper_moveStep(1, 10000, 1);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}