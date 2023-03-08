#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#define TCA9548A_ADDR 0x70       /*!< I2C address of TCA9548A */

#define I2C_MASTER_NUM I2C_NUM_0

void TCA9548A_selectBUS(uint8_t bus) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
    // sender en byte for en bit bliver sat alt efter hvilken bus der bliver brugt
    i2c_master_write_byte(cmd, (1<<bus), true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE("TCA9548A", "Error selecting bus: %d", bus);
    }
    i2c_cmd_link_delete(cmd);
}
