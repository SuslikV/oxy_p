#ifndef PINS_CONFIG_H_INCLUDED
#define PINS_CONFIG_H_INCLUDED

/*
 * Pins physical connections of the ATmega88PA MCU for Oxy_p oximeter
 */

#include <avr/io.h> // MCU Registers

//!!! All power up outputs connected to the same port !!!

// The display hardware powering up pin
#define DEV_1_POWER_PORT 0xD // 0x + "Letter" of the port (0xB..0xF)
#define DEV_1_POWER_BIT 2 // Bit number of the pin (0..7 for the port)
#define DEV_1_POWER_MSK (1 << DEV_1_POWER_BIT)

// All powers on mask
#define ALL_DEV_POWER_MSK (DEV_1_POWER_MSK)

//!!! All buttons connected to the same port !!!

// Left button
#define BUTTON_1_PORT 0xB // 0x + "Letter" of the port (0xB..0xF)
#define BUTTON_1_BIT 4 // Bit number of the pin (0..7 for the port)

// Right button
//
#define BUTTON_2_BIT 5 // to remove set it to BUTTON_1_BIT

#define BUTTON_1_MSK (1 << BUTTON_1_BIT)
#define BUTTON_2_MSK (1 << BUTTON_2_BIT)

// All buttons mask
#define ALL_BTN_MSK (BUTTON_1_MSK | BUTTON_2_MSK)

// Input pins, Interrupt mask, Interrupt enable,
// Direction, Output port registers
#if (DEV_1_POWER_PORT == 0xB)
 #define DEV_POWER_PORT_PINS PINB
 #define DEV_POWER_PCMSK PCMSK0
 #define DEV_POWER_PCIE PCIE0
 #define DEV_POWER_DDR DDRB
 #define DEV_POWER_PORT PORTB
#elif (DEV_1_POWER_PORT == 0xC)
 #define DEV_POWER_PORT_PINS PINC
 #define DEV_POWER_PCMSK PCMSK1
 #define DEV_POWER_PCIE PCIE1
 #define DEV_POWER_DDR DDRC
 #define DEV_POWER_PORT PORTC
#elif (DEV_1_POWER_PORT == 0xD)
 #define DEV_POWER_PORT_PINS PIND
 #define DEV_POWER_PCMSK PCMSK2
 #define DEV_POWER_PCIE PCIE2
 #define DEV_POWER_DDR DDRD
 #define DEV_POWER_PORT PORTD
#endif

#if (BUTTON_1_PORT == 0xB)
 #define BTN_PORT_PINS PINB
 #define BTN_PCMSK PCMSK0
 #define BTN_PCIE PCIE0
 #define BTN_DDR DDRB
 #define BTN_PORT PORTB
#elif (BUTTON_1_PORT == 0xC)
 #define BTN_PORT_PINS PINC
 #define BTN_PCMSK PCMSK1
 #define BTN_PCIE PCIE1
 #define BTN_DDR DDRC
 #define BTN_PORT PORTC
#elif (BUTTON_1_PORT == 0xD)
 #define BTN_PORT_PINS PIND
 #define BTN_PCMSK PCMSK2
 #define BTN_PCIE PCIE2
 #define BTN_DDR DDRD
 #define BTN_PORT PORTD
#endif

// Number of the ADC channel (0..7) for battery voltage measurement
// and its bit according to PORT Function Multiplexing.
//! Pin for ADC should be excluded from internal pull-up in the main code
#define ADC_CHANNEL 1
#define ADC_CHANNEL_PORT_BIT 1
#define ADC_CHANNEL_PORT_MSK (1 << ADC_CHANNEL_PORT_BIT)

// Voltage divider config for ADC
//! Pin for divider should be excluded from internal pull-up in the main code
#define ADC_DIV_CONNECT_PORT 0xC // 0x + "Letter" of the port (0xB..0xF)
#define ADC_DIV_CONNECT_BIT 0 // Bit number of the pin (0..7 for the port)
                              // that connects divider to GND.
#define ADC_DIV_CONNECT_MSK (1 << ADC_DIV_CONNECT_BIT)

#if (ADC_DIV_CONNECT_PORT == 0xB)
 #define ADC_DIV_DDR DDRB
 #define ADC_DIV_PORT PORTB
#elif (ADC_DIV_CONNECT_PORT == 0xC)
 #define ADC_DIV_DDR DDRC
 #define ADC_DIV_PORT PORTC
#elif (ADC_DIV_CONNECT_PORT == 0xD)
 #define ADC_DIV_DDR DDRD
 #define ADC_DIV_PORT PORTD
#endif

#endif // PINS_CONFIG_H_INCLUDED
