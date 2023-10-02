/*
 * Configure UART to send/receive short messages
 */

#include <avr/io.h>
#include <stdio.h>

#include "usart_for_debug.h"

// Basic UART config
#define MYUBRR F_CPU/16/BAUD - 1 // Register value for the selected speed

// Configure USART as UART
void USART_Init(unsigned short int ubrr) {
    if (ubrr == 0)
        ubrr = MYUBRR;
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter, no interrupts
#ifdef ACCEPT_TERMINAL_COMMANDS
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
#else
    UCSR0B = (0 << RXEN0) | (1 << TXEN0);
#endif
    // Set frame format: 8data, 2stop bit, async, no parity
    UCSR0C = (1 <<USBS0) | (3 << UCSZ00);
}

int USART_Transmit(char data, FILE *stream) {
#ifdef WINDOWS_TERMINAL
    if (data == '\n') {
        // Recursive call to add carriage return ( \r\n ) for Windows terminal
        USART_Transmit('\r', stream);
    }
#endif
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
    ;
    // Put data into buffer, sends the data
    UDR0 = data;
    return 0;
}

#ifdef ACCEPT_TERMINAL_COMMANDS
int USART_Receive(FILE *stream) {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)))
    ;
    // Get and return received data from buffer
    return UDR0;
}
#endif

#ifdef ACCEPT_TERMINAL_COMMANDS
// Transmit + Receive
FILE myout = FDEV_SETUP_STREAM(USART_Transmit, USART_Receive, _FDEV_SETUP_RW);
#else
// Transmit only
FILE myout = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
#endif
