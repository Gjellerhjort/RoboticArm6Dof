#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_system.h"

// custom components 
#include "stepper_a4988.h"
#include "servo.h"
#include "TCA9548A.h"
#include "PCF8574.h"
#include "AS5600.h"

/*
timer usses
timer 0 stepper driver

timer 2 servo driver

*/

#define LED_PIN 2
#define LED2_PIN 4
#define LED3_PIN 25

// definere servo pin
#define SERVO_PIN 25

// definere Stepper Step pin
#define STEPPER_1_STEP 3 //defining STEP pin of first motor
#define STEPPER_2_STEP 18    
#define STEPPER_3_STEP 17
#define STEPPER_4_STEP 4
#define STEPPER_5_STEP 15

// definere Stepper DIR pin
#define STEPPER_1_DIR 19 //defining DIR pin of first motor
#define STEPPER_2_DIR 5
#define STEPPER_3_DIR 16
#define STEPPER_4_DIR 2
#define STEPPER_5_DIR 0

#define LED_DELAY 1000
// MicroStepping
#define MS1 14
#define MS2 12
#define MS3 13

#define I2C_SLAVE_ADDR	0x20
#define TIMEOUT_MS		1000
#define DELAY_MS		1000

#define PCF8574_ADDR 0x20
#define TCA9548A_ADDR 0x70


uint8_t rx_data[8];

void i2c_scan()
{
    // i2c init & scan
    for (uint8_t i = 1; i < 127; i++)
    {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI("Found device at:0x", "%2x", i);
        }
    }
}

void led2_callback(void *arg)
{
    while (1) {
        //i2c_scan();
        //servo_move(90);
        stepper_moveStep(1, 200, 0);
        stepper_moveStep(2, 200, 0);
        stepper_moveStep(3, 200, 0);
        stepper_moveStep(4, 200, 0);
        stepper_moveStep(5, 200, 1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        stepper_moveStep(1, 200 , 1);
        stepper_moveStep(2, 200, 1);
        stepper_moveStep(3, 200, 1);
        stepper_moveStep(4, 200, 1);
        stepper_moveStep(5, 200, 1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
void app_main() 
{
    char *Tag = pcTaskGetName(NULL);
    ESP_LOGI(Tag, "Hello Starting\n");
    
    gpio_reset_pin(LED_PIN);
    gpio_reset_pin(LED2_PIN);
    gpio_reset_pin(LED3_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);

    stepper_config(1, STEPPER_1_STEP, STEPPER_1_DIR);
    stepper_config(2, STEPPER_2_STEP, STEPPER_2_DIR);
    stepper_config(3, STEPPER_3_STEP, STEPPER_3_DIR);
    stepper_config(4, STEPPER_4_STEP, STEPPER_4_DIR);
    stepper_config(5, STEPPER_5_STEP, STEPPER_5_DIR);
    microstepping_config(MS1, MS2, MS3);
    stepper_Init();
    stepper_setSpeed(1800);
    //servo_init();

    PCF8574_Init();
    //TCA9548A_Init();
    
    xTaskCreatePinnedToCore(
        led2_callback, // function call name
        "blink_led2_task", // taksk name
        2048, // stack size
        NULL, // task paramerters
        5, // task proirity
        NULL, // task handler 
        1 // CPU core 
    );

    while(1) 
    {
        ESP_LOGI(Tag,"BUS 1:");
        TCA9548A_selectBUS(0);
        AS5600_read();
        i2c_scan();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }   

}