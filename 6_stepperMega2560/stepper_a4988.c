#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"

// definere timer variables
volatile uint16_t timer_period = 1000; // 1ms period
#define TIMER_PRESCALER 8
#define TIMER_PERIOD (F_CPU / TIMER_PRESCALER / timer_period)  // 1ms period

typedef struct StepperInfo {
    // a4988 pins
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t ms1_pin;
    uint8_t ms2_pin;
    uint8_t ms3_pin;
    // stepper variables
    float acceleration;
    volatile uint8_t step_res; // step size resolution (microstepping)
    volatile uint16_t targetPos; // steps that the stepper have to perform
    volatile uint16_t currentPos; // steps done by the stepper
    volatile uint8_t step_direction; // what direction the stepper is rotating
    volatile float speed; // current speed interval in US
} StepperInfo;

// antal stepper i ens kredsløb
#define NUM_STEPPERS 5

volatile StepperInfo steppers[NUM_STEPPERS];

/* Function declarations */
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

// sets pins and configure and starts timer
void stepper_Init()
{
    // Set up timer
    TCCR1B |= (1<<WGM12); // CTC mode
    OCR1A = 2499; // set compare value for 1 sec = 16000000 / (64 * 8.333333333333334) - 1 (must be <65536)
    TIMSK1 |= (1<<OCIE1A); // enable timer compare interrupt
    TCCR1B |= (1 << CS11); // start timer with prescaler = 8
    sei(); //enable global interrupt
}

void stepper_config(uint8_t step_num, uint8_t step_pin, uint8_t dir_pin)
{
    uint8_t i = step_num-1;
    steppers[i].step_pin = step_pin;
    steppers[i].dir_pin = dir_pin;
    steppers[i].currentPos = 0;
    steppers[i].step_direction = 1;
    steppers[i].targetPos = 0;
    if (i<4)
    {
        UART_TxString("porta stepper");
        UART_Newline();
        DDRA |= (1 << step_pin) | (1 << dir_pin);
    }
    else
    {
        UART_TxString("portc stepper");
        UART_Newline();
        DDRC |= (1 << step_pin) | (1 << dir_pin);
    }
    _delay_ms(100);
}

void microstepping_config(uint8_t ms1_pin, uint8_t ms2_pin, uint8_t ms3_pin)
{
    for(int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].ms1_pin = ms1_pin;
        steppers[i].ms2_pin = ms2_pin;
        steppers[i].ms3_pin = ms3_pin;   
    }
    DDRC |= (1 << ms1_pin) | (1 << ms2_pin) | (1 << ms3_pin);

}

void stepper_setMircostepping(uint8_t motor_num ,uint8_t res){
    switch (res)
    {
        case 2: // Half step
            PORTC |= (1<<steppers[motor_num].ms1_pin);
            PORTC &= ~(1<<steppers[motor_num].ms2_pin);
            PORTC &= ~(1<<steppers[motor_num].ms3_pin);
            break;

        case 4: // 1/4 step
            PORTC &= ~(1<<steppers[motor_num].ms1_pin);
            PORTC |= (1<<steppers[motor_num].ms2_pin);
            PORTC &= ~(1<<steppers[motor_num].ms3_pin);
            break;

        case 8: // 1/8 step
            PORTC |= (1<<steppers[motor_num].ms1_pin);
            PORTC |= (1<<steppers[motor_num].ms2_pin);
            PORTC &= ~(1<<steppers[motor_num].ms3_pin);
            break;

        case 16: // 1/16 step
            PORTC |= (1<<steppers[motor_num].ms1_pin);
            PORTC |= (1<<steppers[motor_num].ms2_pin);
            PORTC |= (1<<steppers[motor_num].ms3_pin);
            break;

        default:
            PORTC &= ~(1<<steppers[motor_num].ms1_pin);
            PORTC &= ~(1<<steppers[motor_num].ms2_pin);
            PORTC &= ~(1<<steppers[motor_num].ms3_pin);
    }
}

// Timer Interrupt Service Routine
ISR(TIMER1_COMPA_vect) 
{
    // motor uno
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if(steppers[i].currentPos < steppers[i].targetPos){
                
            if (steppers[i].step_res != 0)
            {
                stepper_setMircostepping(i, steppers[i].step_res);
                steppers[i].step_res = 0;
            }
       
            if (i<4)
            {
                PORTA = (PORTA & ~(1 << steppers[i].dir_pin)) | (steppers[i].step_direction << steppers[i].dir_pin);

                // makes step
                PORTA |= (1<<steppers[i].step_pin); //generate step
                PORTA &= ~(1<<steppers[i].step_pin);
            }
            else
            {
                PORTC = (PORTC & ~(1 << steppers[i].dir_pin)) | (steppers[i].step_direction << steppers[i].dir_pin);
                PORTC |= (1<<steppers[i].step_pin); //generate step
                PORTC &= ~(1<<steppers[i].step_pin);
            }

            steppers[i].currentPos++;
        } else {
            if (i<4)
            {
                PORTA &= ~(1<<steppers[i].step_pin);
            }
            else 
            {
                PORTC &= ~(1<<steppers[i].step_pin);
            }
        }
    }
}

// funktion der sætter hastigheden stepperne laver et nyt step
void stepper_speed(uint16_t new_timer_period) 
{
    // her sætter vi parameteren ligmed vores parametern ew_timer_period
    timer_period = new_timer_period;
    // regner comapare value ud med prescaler 8 og sætter værdien at compare value dette er i hz.
    OCR1A = F_CPU / (8 * timer_period) - 1; 
}

void stepper_move2000(uint8_t direction)
    {
        steppers[0].targetPos = 2000;
        steppers[0].currentPos = 0;
        steppers[0].step_direction = direction;
    }

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    // motor 1A steps
    steppers[motor_num-1].targetPos = steps;
    steppers[motor_num-1].currentPos = 0;
    steppers[motor_num-1].step_direction = direction;
}

// move micro stepper without linear motion
void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution) {
    // motor 1A steps
    steppers[motor_num-1].targetPos = steps;
    steppers[motor_num-1].currentPos = 0;
    steppers[motor_num-1].step_direction = direction;
    steppers[motor_num-1].step_res = stepping_resolution;
}