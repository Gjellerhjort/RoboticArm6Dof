#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "stepper_a4988.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) 
{
    //UART_Init();
    stepper_Init();
    stepper_speed(1100);
    // dette loop kører forevigt da 1 er true og dette kan ikke ændres
    while(1)
    {
      for(int x = 0; x<=6; x++)
      {
        stepper_moveStep(x, 10000, 1);
        _delay_ms(3000);
      }
    }
    return 0;
}