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
//#include "servo.h"
#include "TCA9548A.h"
#include "PCF8574.h"
//#include "AS5600.h"

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

#define I2C_SLAVE_ADDR	0x20
#define TIMEOUT_MS		1000
#define DELAY_MS		1000

#define I2C_MASTER_NUM I2C_NUM_0

#define PCF8574_ADDR 0x20
#define TCA9548A_ADDR 0x70
#define AS5600_ADDR_BASE 0x36



uint8_t rx_data[8];


void led2_callback(void *arg)
{
    while (1) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
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
    microstepping_config(MS1, MS2, MS3);
    stepper_Init();
    stepper_setSpeed(1800);
    void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution);
    /**
    servo_init();
    **/
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
        // Enable all channels on TCA9548A
        uint8_t data = 0xFF;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x01, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
            // Scan each channel for AS5600 sensors
        for (int channel = 0; channel < 8; channel++) 
        {
            data = 1 << channel;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (TCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, 0x01, true);
            i2c_master_write_byte(cmd, data, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            // Scan for AS5600 sensors on this channel
            int sensor = 0;
                uint8_t addr = AS5600_ADDR_BASE | (channel << 1);
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                i2c_master_stop(cmd);
                esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
                i2c_cmd_link_delete(cmd);

                if (err == ESP_OK) 
                {
                    // AS5600 sensor found on this channel
                    // Read data from the sensor using i2c_master_read()

                    // Set the register address to read from (e.g. angle register)
                    uint8_t reg_addr = 0x0E;
                    cmd = i2c_cmd_link_create();
                    i2c_master_start(cmd);
                    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                    i2c_master_write_byte(cmd, reg_addr, true);
                    i2c_master_stop(cmd);
                    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
                    i2c_cmd_link_delete(cmd);

                    // Read data from the sensor (e.g. angle value)
                    uint8_t angle_data[2];
                    cmd = i2c_cmd_link_create();
                    i2c_master_start(cmd);
                    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
                    i2c_master_read_byte(cmd, &angle_data[0], I2C_MASTER_ACK);
                    i2c_master_read_byte(cmd, &angle_data[1], I2C_MASTER_NACK);
                    i2c_master_stop(cmd);
                    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
                    i2c_cmd_link_delete(cmd);

                    // Process the angle data
                    uint16_t angle = (angle_data[0] << 8) | angle_data[1];
                    uint8_t val = PCF8574_readByte();
                    ESP_LOGI("PCF8574 byte:","%X" , val);
                    ESP_LOGI("Sensor: ", "%d", sensor);
                    ESP_LOGI("Angle: ", "%hu", angle);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    // ...
                }
        }
    }   

}