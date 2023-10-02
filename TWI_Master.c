/**
 * \file
 *
 * \brief Application to generate sample driver to AVRs TWI module
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel micro controller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Source file: Atmel-2564-Using-the-TWI-Module-as-I2C-Master_ApplicationNote_AVR315.zip
 */

/*
 * Modified to ease the REPEATED START usage (detached STOP condition).
 * Modified to detect stalled transfers and signals (reset by timeout + empty message).
 * Modified to make number of re-send attempts in case of fail (message importance).
 *
 * Version: M20230827
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_Master.h"

static unsigned char TWI_buf[TWI_BUFFER_SIZE]; // Transceiver buffer
static unsigned char TWI_msgSize;              // Number of bytes to be transmitted

// Importance of the last message:
// 0 --> don't wait for last transfer to complete;
// n --> wait for last transfer to complete and make (n - 1) re-send attempts.
// n = YXXX_XXXX, Y - is STOP message bit, when set, empty stop message will be transmitted instead of STOP signal.
static unsigned char lastMsgWeight = TWI_MSG_IMP_LOW;

volatile union TWI_statusReg TWI_statusReg = {0}; // TWI_statusReg is defined in TWI_Master.h

// The start value for Shifting detector (from LSB to MSB)
#define NOT_YET_SHIFTED 1
volatile unsigned char byte_shifting = NOT_YET_SHIFTED;


////////////////////////////////////////////////////////////////////////
// Call this function to set up the TWI master to its initial standby state.
// Remember to enable interrupts from the main application after initializing the TWI.
////////////////////////////////////////////////////////////////////////

void TWI_Master_Initialize(void) {
    //TWSR = TWI_NO_STATE | TWI_TWPS; // Set TWPS bits. Driver presumes prescaler to be 0.
    TWBR = TWI_TWBR; // Set bit rate register (Baud rate). TWI_TWBR is defined in TWI_Master.h.
    //TWDR = 0xFF;   // Default content = SDA released --> this sets TWI Write Collision Flag (TWWC)
                     // because TWINT is low.
    TWCR = (1 << TWEN) |                               // TWI Interface enabled
           (0 << TWIE) | (0 << TWINT) |                // Disable Interrupt
           (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
           (0 << TWWC);                                // No collisions detected
}


////////////////////////////////////////////////////////////////////////
// Call this function to disconnect the MCU pins from the TWI bus.
// Any ongoing TWI transmissions terminated.
// The TWI bus pins start to act as usual in/out.
////////////////////////////////////////////////////////////////////////

void TWI_Disable(void) {
    TWCR = (0 << TWEN) |                               // Disable TWI-interface and release the TWI pins
           (0 << TWIE) | (1 << TWINT) |                // Disable Interrupt and clear the flag
           (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
           (0 << TWWC);                                // No collisions detected
}


////////////////////////////////////////////////////////////////////////
// Call this function to test if the TWI_ISR is busy transmitting.
////////////////////////////////////////////////////////////////////////

unsigned char TWI_Transceiver_Busy(void) {
    return (TWCR & (1 << TWIE)); // If TWI Interrupt is enabled then the Transceiver is busy
}


////////////////////////////////////////////////////////////////////////
// Call this function to wait for previous operation to complete.
// The function will hold execution (loop) until the TWI_ISR has
// completed with the previous operation or timed out.
// Returns 0 if transfer (the byte shifting) stalled.
////////////////////////////////////////////////////////////////////////

// Set total wait time for 1 bit transfer x255 times longer (1 bit in ~255 SCL clocks).
#define TWI_BIT_WAIT_xTIMES 255

unsigned char TWI_Wait_to_Complete(void) {
    // 1 byte transfer = (start) + 8 [data] + 1 [ACK] + (stop), TWI bus clocks
    // Also, the bus speed can be dropped by the slave.
    unsigned char timeout = TWI_BIT_WAIT_xTIMES; // in SCL clocks
    while (TWI_Transceiver_Busy() && timeout) {
        // Take a pause for 1 bit shift (single SCL cycle)
        __builtin_avr_delay_cycles(T_LOW_CYCL + T_HIGH_CYCL);
        --timeout;

        //!!! Shifting detector.
        // Analyze if shift happened during the last SCL clocks cycle.
        // If shifting goes on then transfer is running OK.
        // The 0x00 data byte at TWDR has same shift result, so cannot be analyzed.
        if (byte_shifting != TWDR) {
            // Reset analyzer, reset timeout.
            byte_shifting = TWDR;
            timeout = TWI_BIT_WAIT_xTIMES;
        }
    }
    return timeout;
}


////////////////////////////////////////////////////////////////////////
// Call this function to fetch the status of the previous operation. The function will hold execution (loop)
// until the TWI_ISR has completed with the previous operation or timed out. If there was an error, then the function
// will free the TWI bus and return FALSE (0).
////////////////////////////////////////////////////////////////////////

static unsigned char TWI_Get_Last_Trans_Status(void) {
    if (TWI_Wait_to_Complete() == 0) {
        TWI_Disable(); // Free the bus if stalled (uncatchable bus errors)
        TWI_statusReg.lastTransOK = FALSE;
    };
    return TWI_statusReg.lastTransOK;
}


////////////////////////////////////////////////////////////////////////
// Workaround for ATmega88PA+MAX30102+SSD1306 I have (chinese clones),
// after last byte received by master + NACK->STOP - stop cannot complete (TWSTO remains set),
// but ...NACK->START->OTHER_ADDR_RW(NACK)->STOP can complete normally.
// Release bus pins also will work fine, and even faster. But behavior of the slave device
// in this case will be unpredictable.
////////////////////////////////////////////////////////////////////////

static void TWI_Empty_Message(void) {
    TWI_msgSize = 1; // Shorten last message to its own address
    TWI_buf[0] = TWI_SOME_FREE_ADDR_WR; // Write call, no respond -> lastTransOK = FALSE
}


////////////////////////////////////////////////////////////////////////
// Call this function to complete transfer of the last message and send STOP signal.
////////////////////////////////////////////////////////////////////////

unsigned char TWI_Complete_Last_Transfer(unsigned char send_stop) {
    unsigned char temp;
    temp = lastMsgWeight & TWI_MSG_IMP_MASK; // (n - 1) of attempts to re-send previous message, counts down to 0
    while (temp) {
        --temp;
        if (TWI_Get_Last_Trans_Status()) {
            break;
        } else if (temp) {
            TWI_Start_Transceiver();
        }
    };

    if (TWI_statusReg.lastTransOK && send_stop) {
        if (lastMsgWeight & TWI_MSG_STOP) { // Test if empty message should be transmitted
            TWI_Empty_Message();
            TWI_Start_Transceiver();
            TWI_Wait_to_Complete();
            TWI_statusReg.lastTransOK = TRUE; // Ignore empty message results
        }
        TWI_STOP_Signal();
    }
    return TWI_statusReg.lastTransOK;
}


////////////////////////////////////////////////////////////////////////
// Call this function to send a prepared message. The first byte must contain the slave address and the
// read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
// from the slave. Also include how many bytes that should be sent/read including the address byte.
// The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
// then initialize the next operation and return. Message importance should be specified.
////////////////////////////////////////////////////////////////////////

void TWI_Start_Transceiver_With_Data(unsigned char *msg, unsigned char msgSize, unsigned char msgImp) {
    unsigned char temp;

    TWI_Complete_Last_Transfer(IGNORE_STOP); // REPEATED start expected

    lastMsgWeight = msgImp;

#ifdef TWI_FORCE_EMPTY_MSG_BEFORE_STOP
    lastMsgWeight += TWI_MSG_STOP;
#endif

    TWI_msgSize = msgSize;                    // Number of data to transmit (with address)
    TWI_buf[0] = msg[0];                      // Store slave address with R/W setting
    if (!(msg[0] & (TRUE << TWI_READ_BIT))) { // If it is a write operation, then also copy data
        for (temp = 1; temp < msgSize; temp++) {
            TWI_buf[temp] = msg[temp];
        }
    }

    TWI_statusReg.all = 0;
    TWCR = (1 << TWEN) |                               // TWI Interface enabled
           (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag
           (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a START condition
           (0 << TWWC);                                // No collisions detected
}


////////////////////////////////////////////////////////////////////////
// Call this function to resend the last message. The driver will reuse the data previously
// put in the transceiver buffer.
////////////////////////////////////////////////////////////////////////

void TWI_Start_Transceiver(void) {
    TWI_statusReg.all = 0;
    TWCR = (1 << TWEN) |                               // TWI Interface enabled
           (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag
           (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a START condition
           (0 << TWWC);                                // No collisions detected
}


////////////////////////////////////////////////////////////////////////
// Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
// TWI_Start_Transceiver_With_Data to send a request for data to the slave. Then Run this function to collect the
// data when they have arrived. Previous call to TWI_Start_Transceiver_With_Data completes with the STOP signal
// (this needed to free the slave as fast as possible).
// Include a pointer to where to place the data and the number of bytes
// requested (including the address field) in the function call. The function will hold execution (loop)
// until the TWI_ISR has completed with the previous operation, before reading out the data and returning.
// If there was an error in the previous transmission the function will return the TWI error code (buffer data
// not modified).
//
// NOTE: When the internal read pointer of the device increments automatically - the last read message
// shouldn't be re-send, instead whole sequence should repeat: write_from_where_to_read->read_from.
////////////////////////////////////////////////////////////////////////

unsigned char TWI_Get_Data_From_Transceiver(unsigned char *msg, unsigned char msgSize) {
    unsigned char i;
    if (TWI_Complete_Last_Transfer(SEND_STOP)) {
        // Last transmission competed successfully,
        // copy data from Transceiver buffer.
        for (i = 0; i < msgSize; i++) {
            msg[i] = TWI_buf[i];
        }
    }
    return (TWI_statusReg.lastTransOK);
}


////////////////////////////////////////////////////////////////////////
// Call this function to send STOP signal.
//
// Sends the STOP with timeout. Waiting for STOP to complete.
// For example, the SSD1306 may hung the MCU TWI (TWSTO bit remains "1" forever)
// just after the START->ADDR_WR->ACK->STOP sequence.
// ---------------------------------------------------------------------
// Requires: TWINT bit already should be set by MCU hardware
// (the SCL low period is stretched)!!!
//
// NOTE: The normal timeout value T_BUF_CYCL + T_SU_STO_CYCL has robust
// results in real applications with single slave. For example, normally (no hung),
// the 8MHz MCU with 400kHz I2C bus will complete the STOP signal
// within ~13 counts of MCU clocks (was tested), while the T_BUF_CYCL + T_SU_STO_CYCL = ~16 MCU clocks
// (13 fits in, thus signal is not ends abrupt).
// With 2 slaves + empty stop message the STOP signal ends in about 6..8 SCL clocks (was tested).
////////////////////////////////////////////////////////////////////////

// Set total wait time for STOP signal x255 times longer (~255 SCL clocks).
#define TWI_STOP_WAIT_xTIMES 255

void TWI_STOP_Signal(void) {
    TWCR = (1 << TWEN) |                               // TWI Interface enabled
           (0 << TWIE) | (1 << TWINT) |                // Disable TWI Interrupt and clear the flag
           (0 << TWEA) | (0 << TWSTA) | (1 << TWSTO) | // Initiate a STOP condition
           (0 << TWWC);                                // No collisions detected

    unsigned char timeout;
    timeout = TWI_STOP_WAIT_xTIMES;

    // Wait for STOP to complete or end it by timeout.
    while((TWCR & (1 << TWSTO)) && timeout) {
        // Take a pause for length of 1 STOP signal (about 1 SCL cycle)
        __builtin_avr_delay_cycles(T_BUF_CYCL + T_SU_STO_CYCL);
        --timeout;
    }

    // When STOP ended by timeout - free the bus
    if (!timeout) {
        TWI_Disable();
    }
}


/********** Interrupt Handlers **********/

////////////////////////////////////////////////////////////////////////
// This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
// that is whenever a TWI event has occurred. This function should not be called directly from the main
// application.
////////////////////////////////////////////////////////////////////////

ISR (TWI_vect) {
    static unsigned char TWI_bufPtr;

    switch (TWSR) {
        case TWI_START:        // START has been transmitted,
        case TWI_REP_START:    // Repeated START has been transmitted.
            TWI_bufPtr = 0;    // Set buffer pointer to the TWI Address location
        case TWI_MTX_ADR_ACK:    // SLA+W has been transmitted and ACK received,
        case TWI_MTX_DATA_ACK:   // Data byte has been transmitted and ACK received.
            if (TWI_bufPtr < TWI_msgSize) {
                byte_shifting = NOT_YET_SHIFTED; // Reset Shifting detector
                TWDR = TWI_buf[TWI_bufPtr++];
                TWCR = (1 << TWEN) |                               // TWI Interface enabled
                       (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag to send byte
                       (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
                       (0 << TWWC);                                // No collisions detected
            } else {
                TWI_statusReg.lastTransOK = TRUE;                  // Set status bits to completed successfully
                                    // Prepare to send STOP or REPEATED START after last byte but do not send any signals
                TWCR = (1 << TWEN) |                               // TWI Interface enabled
                       (0 << TWIE) | (0 << TWINT) |                // Disable Interrupt and preserve the flag. While the TWINT is set, the SCL low period is stretched.
                       (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
                       (0 << TWWC);
            }
            break;

        case TWI_MRX_DATA_ACK: // Data byte has been received and ACK transmitted
            TWI_buf[TWI_bufPtr++] = TWDR;
            byte_shifting = NOT_YET_SHIFTED; // Reset Shifting detector
        case TWI_MRX_ADR_ACK:  // SLA+R has been transmitted and ACK received
            if (TWI_bufPtr < (TWI_msgSize - 1)) {                  // Detect the last byte to NACK it
                TWCR = (1 << TWEN) |                               // TWI Interface enabled
                       (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag to read next byte
                       (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // Send ACK after reception
                       (0 << TWWC);                                // No collisions detected
            } else {             // Send NACK after next reception
                TWCR = (1 << TWEN) |                               // TWI Interface enabled
                       (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag to read next byte
                       (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // Send NACK after reception
                       (0 << TWWC);                                // No collisions detected
            }
            break;

        case TWI_MRX_DATA_NACK: // Data byte has been received and NACK transmitted
            TWI_buf[TWI_bufPtr] = TWDR;
            TWI_statusReg.lastTransOK = TRUE;                  // Set status bits to completed successfully
                                // Prepare to send STOP or REPEATED START after last byte but do not send any signals
            TWCR = (1 << TWEN) |                               // TWI Interface enabled
                   (0 << TWIE) | (0 << TWINT) |                // Disable Interrupt and preserve the flag. While the TWINT is set, the SCL low period is stretched.
                   (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
                   (0 << TWWC);
            break;

        case TWI_ARB_LOST:      // Arbitration lost
            TWCR = (1 << TWEN) |                               // TWI Interface enabled
                   (1 << TWIE) | (1 << TWINT) |                // Enable TWI Interrupt and clear the flag
                   (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a (RE)START condition
                   (0 << TWWC);                                // No collisions detected
            break;

        case TWI_BUS_ERROR:     // Bus error due to an illegal START or STOP condition.
            TWI_STOP_Signal(); // No stop signal will be generated! SDA and SCL lines are released.
            break;

        case TWI_MTX_ADR_NACK:  // SLA+W has been transmitted and NACK received,
        case TWI_MRX_ADR_NACK:  // SLA+R has been transmitted and NACK received,
        case TWI_MTX_DATA_NACK: // Data byte has been transmitted and NACK received.
        default:
                                // Prepare to send STOP or REPEATED START after last byte but do not send any signals
            TWCR = (1 << TWEN) |                               // TWI Interface enabled
                   (0 << TWIE) | (0 << TWINT) |                // Disable Interrupt and preserve the flag. While the TWINT is set, the SCL low period is stretched.
                   (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
                   (0 << TWWC);                                // No collisions detected
    }
}
