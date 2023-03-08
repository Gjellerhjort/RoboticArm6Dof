#include "driver/gpio.h"
#include "driver/i2c.h"

// Define the AS5600 sensor I2C address
#define AS5600_SENSOR_ADDR 0x36

// Define the AS5600 sensor command for reading the angle measurement
#define AS5600_ANGLE_CMD 0x0C

#define I2C_MASTER_NUM I2C_NUM_0

uint16_t AS5600_read()
{
    // Initialize the AS5600 sensor
    uint8_t angle_cmd[1] = {AS5600_ANGLE_CMD};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, angle_cmd, 1, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Error Initialize: %d", err);
    } 
    else
    {
        i2c_cmd_link_delete(cmd);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // Read data from the AS5600 sensor
        uint8_t angle_data[2];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AS5600_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, angle_data, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t error = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        if (error != ESP_OK) {
            ESP_LOGE("AS5600", "Error Reading: %d", error);
        }
        i2c_cmd_link_delete(cmd);
        // Print the angle measurement
        uint16_t angle = (angle_data[0] << 8) | angle_data[1];
        ESP_LOGI("Angle measurement", "%u", angle);
        return angle;
    }
    return 0;
}