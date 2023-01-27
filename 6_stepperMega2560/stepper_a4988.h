/******************************************
* "stepper_a4988.H":                      *
* Header file for Arduino a4988 driver    *
*                                         *
*******************************************/ 
#include "stepper_a4988.c"

// sets pins for stepper as output and starts the timer iterupts for stepper
void stepper_Init();

// set the time between each step the higher time period the slower the stepper will move
void stepper_speed(uint16_t new_timer_period);

//void stepper_moveDeg(uint8_t );
void stepper_move2000(uint8_t direction);
// function that moves a stepper 200 steps the given direction
void stepper_moveStep(uint8_t motor_num, uint16_t steps, uint8_t direction);

/******************************************/
