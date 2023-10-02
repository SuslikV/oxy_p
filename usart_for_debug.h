#ifndef USART_FOR_DEBUG_H_INCLUDED
#define USART_FOR_DEBUG_H_INCLUDED

/*
 * Headers file for the UART to send/receive short messages
 */

// Basic UART config
//#define BAUD 9600
//#define BAUD 14400
//#define BAUD 19200
#define BAUD 38400 // Faster transfer fails for my system
#ifndef LINUX_TERMINAL
 #define WINDOWS_TERMINAL // Use CRLF instead of LF in UART transmissions
#endif
//#define ACCEPT_TERMINAL_COMMANDS // Use terminal to send commands to MCU


////////////////////////////////////////////////////////////////////////
// Function definitions
////////////////////////////////////////////////////////////////////////

void USART_Init(unsigned short int);
int USART_Transmit(char, FILE*);
#ifdef ACCEPT_TERMINAL_COMMANDS
int USART_Receive(FILE*);
#endif

#endif // USART_FOR_DEBUG_H_INCLUDED
