#include "uart.c"

void UART_Init();
void UART_TxChar(unsigned char data);
void UART_TxNumber(unsigned int num);
void UART_TxString(char *string_ptr);
void UART_Newline();