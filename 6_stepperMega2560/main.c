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
#define STEPPER_5_STEP PA0 //defining STEP pin of first motor
#define STEPPER_4_STEP PA2    
#define STEPPER_3_STEP PA4
#define STEPPER_2_STEP PA6
#define STEPPER_1_STEP PC7

// definere Stepper DIR pin
#define STEPPER_5_DIR PA1 //defining DIR pin of first motor
#define STEPPER_4_DIR PA3
#define STEPPER_3_DIR PA5
#define STEPPER_2_DIR PA7
#define STEPPER_1_DIR PC6

// MicroStepping
#define MS1 PC5
#define MS2 PC4
#define MS3 PC3

void test1()
{
  stepper_moveStep(1, 400, 1);
  stepper_moveStep(2, 800, 1);
  stepper_moveStep(3, 800, 1);
  stepper_moveStep(4, 50, 1);
  stepper_moveStep(5, 50, 1);
  _delay_ms(4000);
  stepper_moveStep(1, 400, 0);
  stepper_moveStep(2, 800, 0);
  stepper_moveStep(3, 800, 0);
  stepper_moveStep(4, 50, 0);
  stepper_moveStep(5, 50, 0);
  _delay_ms(4000);
  
}

void test2()
{
  servo_set_position(0);
  _delay_ms(3000);
  servo_set_position(180);
  _delay_ms(3000);
}

int main(void) 
{
    UART_Init();
    stepper_Init();
    servo_init();
    stepper_config(1, STEPPER_1_STEP, STEPPER_1_DIR, 10200);
    stepper_config(2, STEPPER_2_STEP, STEPPER_2_DIR, 10200);
    stepper_config(3, STEPPER_3_STEP, STEPPER_3_DIR, 3800);
    stepper_config(4, STEPPER_4_STEP, STEPPER_4_DIR, 200);
    stepper_config(5, STEPPER_5_STEP, STEPPER_5_DIR, 200);
    microstepping_config(MS1, MS2, MS3);
    stepper_speed(500);
    // dette loop kører forevigt da 1 er true og dette kan ikke ændres
    DDRA |= (1 << PA6) | (1 << PA7);
    DDRC |= (1 << PC6) | (1 << PC7);
    while(1)
    {
      stepper_moveStep(1, 400, 0);
      //stepper_moveStep(2, 800, 0);
      stepper_moveStep(3, 2549, 0);
      stepper_moveMicrostep(4, 100, 1, 4);
      stepper_moveMicrostep(5, 400, 1, 8);
      servo_set_position(0);
      _delay_ms(6000);
      stepper_moveStep(1, 400, 1);
      //stepper_moveStep(2, 800, 1);
      stepper_moveStep(3,2549, 1);      
      stepper_moveMicrostep(4, 100, 0, 4);
      stepper_moveMicrostep(5, 400, 0, 8);
      servo_set_position(180);
      _delay_ms(6000); 
    }
}