#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#define TCA9548A_ADDR 0x70       /*!< I2C address of TCA9548A */

uint8_t TCA9548A_readByte(uint8_t i2c_num, uint8_t reg_addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (1 << i2c_num) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (1 << i2c_num) | I2C_MASTER_READ, true);
    uint8_t input_val;
    i2c_master_read_byte(cmd, &input_val, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    uint8_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}