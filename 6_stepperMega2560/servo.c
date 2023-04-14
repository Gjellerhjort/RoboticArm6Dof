#include <avr/io.h>

#define SERVO_PIN PE3 // The pin connected to the servo

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_init()
{
    TCCR3A = (1 << COM3A1) | (1 << WGM31); // Set PWM output mode and mode 14 (fast PWM)
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // Set mode 14 (fast PWM) and prescaler to 8
    ICR3 = 20000; // Set TOP value for timer 1
    DDRE |= (1 << SERVO_PIN); // Set the servo pin as output
}

void servo_set_position(int deg)
{
    OCR3A = map(deg,0,180,2000,4000);
}

