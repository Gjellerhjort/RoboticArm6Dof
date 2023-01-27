#include <util/setbaud.h>

void UART_Init()
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}

void UART_TxChar(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_TxNumber(unsigned int num)
{
 
       UART_TxChar((num/10000)+0x30);
       num=num%10000;
 
       UART_TxChar((num/1000)+0x30);
       num=num%1000;
 
       UART_TxChar((num/100)+0x30);
       num=num%100;
 
       UART_TxChar((num/10)+0x30);
 
       UART_TxChar((num%10)+0x30);
}

void UART_TxString(char *string_ptr)
{
          while(*string_ptr)
           UART_TxChar(*string_ptr++);
}

void UART_Newline()
{
    UART_TxChar(0x0a);
    UART_TxChar(0x0d);
}

/**
void SerialWrite(char data[20])
{
    int i = 0;
    while(data[i]!='\0') { // takes every charters in the string and sends it 
    UART_Transmit(data[i]);
    i++;
    }
    UART_Transmit(0x0a); // newline
    UART_Transmit(0x0d); // sets cursor left 
}
**/