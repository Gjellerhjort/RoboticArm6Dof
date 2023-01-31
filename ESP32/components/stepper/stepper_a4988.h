#include "stepper_a4988.c"
#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
// when included in C++ file, let compiler know these are C functions
extern "C" {
#endif

void call_stepper();
void stepper_Init();
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction);
void stepper_setSpeed(uint16_t new_timer_period);

#ifdef __cplusplus
}
#endif
 


#endif // stepper_H