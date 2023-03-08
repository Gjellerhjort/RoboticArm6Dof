#include "TCA9548A.c"

uint8_t TCA9548A_readByte(uint8_t i2c_num, uint8_t reg_addr);
void as5600_read(uint8_t channel);
void TCA9548A_select_channel(uint8_t channel);
void TCA9548A_Init();