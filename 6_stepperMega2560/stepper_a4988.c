#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// uno test
#define STEPPER_0_STEP PD6 // pin for direction control on A4988 driver
#define STEPPER_0_DIR PD5 // pin for step control on A4988 driver
volatile uint16_t stepper_0_count = 2000;
volatile uint8_t stepper_0_direction = 1;

// definere Stepper Step pin
#define STEPPER_1_STEP PA1 //defining STEP pin of first motor
#define STEPPER_2_STEP PA3    
#define STEPPER_3_STEP PA5
#define STEPPER_4_STEP PA7
#define STEPPER_5_STEP PC6
#define STEPPER_6_STEP PC4

// definere Stepper DIR pin
#define STEPPER_1_DIR PA0 //defining DIR pin of first motor
#define STEPPER_2_DIR PA2
#define STEPPER_3_DIR PA4
#define STEPPER_4_DIR PA6
#define STEPPER_5_DIR PC7
#define STEPPER_6_DIR PC5

// definere timer variables
volatile uint16_t timer_period = 1000; // 1ms period
#define TIMER_PRESCALER 8
#define TIMER_PERIOD (F_CPU / TIMER_PRESCALER / timer_period)  // 1ms period


// definere stepper variables
volatile uint16_t stepper_1_count = 0; // dette er hvor mange steps stepper 1 skal udfører
volatile uint8_t stepper_1_direction = 1; // dette er retning motoren skal bevæge sig
volatile uint16_t stepper_2_count = 0;
volatile uint8_t stepper_2_direction = 1;
volatile uint16_t stepper_3_count = 0;
volatile uint8_t stepper_3_direction = 1;
volatile uint16_t stepper_4_count = 0;
volatile uint8_t stepper_4_direction = 1;
volatile uint16_t stepper_5_count = 0;
volatile uint8_t stepper_5_direction = 1;
volatile uint16_t stepper_6_count = 0;
volatile uint8_t stepper_6_direction = 1;

// sets pins and configure and starts timer
void stepper_Init()
{
    // Set output pins for stepper 0
    //DDRD |= (1 << STEPPER_0_STEP) | (1 << STEPPER_0_DIR);
    // Set output pins for stepper 1
    DDRA |= (1 << STEPPER_1_STEP) | (1 << STEPPER_1_DIR);
    // Set output pins for stepper 2
    DDRA |= (1 << STEPPER_2_STEP) | (1 << STEPPER_2_DIR);
    // Set output pins for stepper 3
    DDRA |= (1 << STEPPER_3_STEP) | (1 << STEPPER_3_DIR);
    // Set output pins for stepper 4
    DDRA |= (1 << STEPPER_4_STEP) | (1 << STEPPER_4_DIR);
    // Set output pins for stepper 5
    DDRC |= (1 << STEPPER_5_STEP) | (1 << STEPPER_5_DIR);
    // Set output pins for stepper 6
    DDRC |= (1 << STEPPER_6_STEP) | (1 << STEPPER_6_DIR);
    
    // Set up timer
    TCCR1B |= (1<<WGM12); // CTC mode
    OCR1A = 2499; // set compare value for 1 sec = 16000000 / (64 * 8.333333333333334) - 1 (must be <65536)
    TIMSK1 |= (1<<OCIE1A); // enable timer compare interrupt
    TCCR1B |= (1 << CS11); // start timer with prescaler = 8
    sei(); //enable global interrupt

} 

// Timer Interrupt Service Routine
ISR(TIMER1_COMPA_vect) 
{
    // motor uno
    /**
    if(stepper_0_count > 0){
        PORTD = (PORTD & ~(1 << STEPPER_0_DIR)) | (stepper_0_direction << STEPPER_0_DIR);
        PORTD |= (1<<STEPPER_0_STEP); //generate step
        PORTD &= ~(1<<STEPPER_0_STEP);
        stepper_0_count--;
    } else
    {
        PORTD &= ~(1 << STEPPER_0_STEP);
    }
    */
    // motor 1A
    if(stepper_1_count > 0){
        PORTA = (PORTA & ~(1 << STEPPER_1_DIR)) | (stepper_1_direction << STEPPER_1_DIR);
        PORTA |= (1<<STEPPER_1_STEP); //generate step
        PORTA &= ~(1<<STEPPER_1_STEP);
        stepper_1_count--;
    } else
    {
        PORTA &= ~(1 << STEPPER_1_STEP);
    }

    // motor 2A
    if(stepper_2_count > 0){
        PORTA = (PORTA & ~(1 << STEPPER_2_DIR)) | (stepper_2_direction << STEPPER_2_DIR);
        PORTA |= (1<<STEPPER_2_STEP); //generate step
        PORTA &= ~(1<<STEPPER_2_STEP);
        stepper_2_count--;
    } else
    {
        PORTA &= ~(1 << STEPPER_2_STEP);
    }

    // motor 3A

    if(stepper_3_count > 0){
        PORTA = (PORTA & ~(1 << STEPPER_3_DIR)) | (stepper_3_direction << STEPPER_3_DIR);
        PORTA |= (1<<STEPPER_3_STEP); //generate step
        PORTA &= ~(1<<STEPPER_3_STEP);
        stepper_3_count--;
    } else
    {
        PORTA &= ~(1 << STEPPER_3_STEP);
    }

    // motor 4A

    if(stepper_4_count > 0){
        PORTA = (PORTA & ~(1 << STEPPER_4_DIR)) | (stepper_4_direction << STEPPER_4_DIR);
        PORTA |= (1<<STEPPER_4_STEP); //generate step
        PORTA &= ~(1<<STEPPER_4_STEP);
        stepper_4_count--;
    } else
    {
        PORTA &= ~(1 << STEPPER_4_STEP);
    }

    // motor 5C
    if(stepper_5_count > 0){
        PORTC = (PORTC & ~(1 << STEPPER_5_DIR)) | (stepper_5_direction << STEPPER_5_DIR);
        PORTC |= (1<<STEPPER_5_STEP); //generate step
        PORTC &= ~(1<<STEPPER_5_STEP);
        stepper_5_count--;
    } else
    {
        PORTC &= ~(1 << STEPPER_5_STEP);
    }

    // motor 6C

    if(stepper_6_count > 0){
        PORTC = (PORTC & ~(1 << STEPPER_6_DIR)) | (stepper_6_direction << STEPPER_6_DIR);
        PORTC |= (1<<STEPPER_6_STEP); //generate step
        PORTC &= ~(1<<STEPPER_6_STEP);
        stepper_6_count--;
    } else
    {
        PORTC &= ~(1 << STEPPER_6_STEP);
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
        stepper_1_count = 2000;
        stepper_1_direction = direction;
    }

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    // motor 1A steps
    if (motor_num == 1) // checker om det er motor 1A
    { 
        stepper_1_count = steps; // sætter antal steps som motoren skal bevæge sig  
        stepper_1_direction = direction; // sætter retning motoren skal bevæge sig
    } else if (motor_num == 2)  // motor 2A 200
    { 
        stepper_2_count = steps;   
        stepper_2_direction = direction;
    } else if (motor_num == 3) // motor 3A 200
    {
        stepper_3_count = steps;   
        stepper_3_direction = direction;
    } else if (motor_num == 4) // motor 4A 200
    {
        stepper_4_count = steps;  
        stepper_4_direction = direction;
    } else if (motor_num == 5) // motor 5C 200
    {
        stepper_5_count = steps;  
        stepper_5_direction = direction;
    } else if (motor_num == 6) // motor 6C 200
    {
        stepper_6_count = steps;  
        stepper_6_direction = direction;
    }
}