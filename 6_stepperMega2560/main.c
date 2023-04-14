#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "servo.h"
#include "stepper_a4988.h"
#include "i2c.h"

// definere Stepper Step pin
#define STEPPER_1_STEP PA0 //defining STEP pin of first motor
#define STEPPER_2_STEP PA2    
#define STEPPER_3_STEP PA4
#define STEPPER_4_STEP PA6
#define STEPPER_5_STEP PC7

// definere Stepper DIR pin
#define STEPPER_1_DIR PA1 //defining DIR pin of first motor
#define STEPPER_2_DIR PA3
#define STEPPER_3_DIR PA5
#define STEPPER_4_DIR PA7
#define STEPPER_5_DIR PC6

int main(void) 
{
    UART_Init();
    stepper_Init();
    servo_init();
    stepper_config(1, STEPPER_1_STEP, STEPPER_1_DIR);
    stepper_config(2, STEPPER_2_STEP, STEPPER_2_DIR);
    stepper_config(3, STEPPER_3_STEP, STEPPER_3_DIR);
    stepper_config(4, STEPPER_4_STEP, STEPPER_4_DIR);
    stepper_config(5, STEPPER_5_STEP, STEPPER_5_DIR);
    stepper_speed(120);
    // dette loop kører forevigt da 1 er true og dette kan ikke ændres
    while(1)
    {
      stepper_moveStep(1, 200, 1);
      stepper_moveStep(2, 400, 1);
      stepper_moveStep(3, 1000, 1);
      stepper_moveStep(4, 50, 1);
      stepper_moveStep(5, 100, 1);
      _delay_ms(3000);
      stepper_moveStep(1, 200, 0);
      stepper_moveStep(2, 400, 0);
      stepper_moveStep(3, 1000, 0);
      stepper_moveStep(4, 50, 0);
      stepper_moveStep(5, 100, 0);
      _delay_ms(3000);
      servo_set_position(180);
      _delay_ms(3000);
      servo_set_position(0);
      _delay_ms(3000);
    }
}