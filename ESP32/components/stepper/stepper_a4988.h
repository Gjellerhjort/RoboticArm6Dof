#include "stepper_a4988.c"
#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
// when included in C++ file, let compiler know these are C functions
extern "C" {
#endif

void call_stepper();
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction);
void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution);
void stepper_setSpeed(uint16_t new_timer_period);
void stepper_config(uint8_t step_num, uint8_t step_pin, uint8_t dir_pin);
void microstepping_config(uint8_t ms1_pin, uint8_t ms2_pin, uint8_t ms3_pin);
void prepareMovement(uint8_t motor_num, uint16_t steps);
void runAndWait();

#ifdef __cplusplus
}
#endif
 


#endif // stepper_H