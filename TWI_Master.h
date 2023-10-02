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


#include <avr/builtins.h> // delay in cycles

////////////////////////////////////////////////////////////////////////
// TWI Status/Control register definitions
////////////////////////////////////////////////////////////////////////

// Set this to the largest message size (in bytes) that will be sent including address byte.
// Here 1 byte address + 1 byte command/data + SSD1306 page size of 128 bytes.
// Shouldn't exceed 255.
#define TWI_BUFFER_SIZE (1 + 1 + 128)

// I2C frequency in Hz
#define F_TWI_SCL  400000
//#define F_TWI_SCL  100000

// For ATmega88PA:
// F_TWI_SCL = F_CPU/(16 + 2*TWBR*(4^TWPS))
// thus TWBR = (F_CPU/F_TWI_SCL - 16) / (2*(4^TWPS))

// Set register value to setup desired SCL frequency
// TWBR alone can cover all possible SCL frequencies (up to 400kHz),
// thus the prescaler is not needed (TWPS = 0)
#define TWI_TWPS ((0 << TWPS0) | (0 << TWPS1))

#if (TWI_TWPS > 0)
 #define TWI_4_POWER_OF_TWPS (2 << (2 * TWI_TWPS - 1))
#else
 #define TWI_4_POWER_OF_TWPS 1
#endif

#define TWI_TWBR ((F_CPU / F_TWI_SCL - 16) / (2 * TWI_4_POWER_OF_TWPS))

#if (TWI_TWPS)
 #error "TWI_Master.h requires TWI prescaler (TWPS bits) to be 0"
#endif

// Delays to guaranty the STOP condition timings, in MCU cycles:
//   T_SU_STO_CYCL - Setup time for STOP condition
//   T_SU_STA_CYCL - Setup time for REPEATED START condition
//   T_BUF_CYCL - Bus free time between a STOP and START condition
//   T_LOW_CYCL - Low period of the SCL clock
//   T_HIGH_CYCL - High period of the SCL clock
// Values are rounded up because the number of MCU cycles is integer.
// Max delay (cycles) should not exceed 255 (F_CPU < 54 MHz).
#if (F_TWI_SCL > 100000)
 #define T_SU_STO_CYCL ((F_CPU *  6) / 10000000 + 1) // 0.6 usec
 #define T_SU_STA_CYCL ((F_CPU *  6) / 10000000 + 1) // 0.6 usec
 #define T_BUF_CYCL    ((F_CPU * 13) / 10000000 + 1) // 1.3 usec
 #define T_HIGH_CYCL   ((F_CPU *  6) / 10000000 + 1) // 0.6 usec
 #define T_LOW_CYCL    ((F_CPU * 13) / 10000000 + 1) // 1.3 usec
#else
 #define T_SU_STO_CYCL ((F_CPU * 40) / 10000000 + 1) // 4.0 usec
 #define T_SU_STA_CYCL ((F_CPU * 40) / 10000000 + 1) // 4.0 usec
 #define T_BUF_CYCL    ((F_CPU * 47) / 10000000 + 1) // 4.7 usec
 #define T_HIGH_CYCL   ((F_CPU * 40) / 10000000 + 1) // 4.0 usec
 #define T_LOW_CYCL    ((F_CPU * 47) / 10000000 + 1) // 4.7 usec
#endif

#if ((T_SU_STO_CYCL > 255) || \
     (T_BUF_CYCL    > 255) || \
     (T_HIGH_CYCL   > 255) || \
     (T_LOW_CYCL    > 255))
 #error "TWI_Master.h requires lower F_CPU to fit the TWI timings into 1 byte."
#endif

////////////////////////////////////////////////////////////////////////
// Global definitions
////////////////////////////////////////////////////////////////////////

// Status byte holding flags
union TWI_statusReg {
    unsigned char all;
    struct {
        unsigned char lastTransOK: 1; // 0000000X
        unsigned char unusedBits: 7;  // XXXXXXX0
    };
};


////////////////////////////////////////////////////////////////////////
// Function definitions
////////////////////////////////////////////////////////////////////////

void TWI_Master_Initialize (void);
void TWI_Disable(void);
unsigned char TWI_Transceiver_Busy(void);
unsigned char TWI_Get_State_Info(void);
unsigned char TWI_Wait_to_Complete(void);
void TWI_Start_Transceiver_With_Data(unsigned char * , unsigned char, unsigned char);
void TWI_Start_Transceiver(void);
unsigned char TWI_Complete_Last_Transfer(unsigned char);
unsigned char TWI_Get_Data_From_Transceiver(unsigned char *, unsigned char);
void TWI_STOP_Signal(void);

// Bit and byte definitions

#define TWI_READ_BIT  0 // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1 // Bit position for LSB of the slave address bits in the init byte.

#define TRUE          1
#define FALSE         0

#define TWI_MSG_ABORT 0      // No wait (abort), 0 attempts to re-send
#define TWI_MSG_IMP_LOW 1    // Wait, 0 attempts to re-send
#define TWI_MSG_IMP_NORMAL 2 // Wait, 1 attempts to re-send
#define TWI_MSG_IMP_HIGH 11  // Wait, 10 attempts to re-send
#define TWI_MSG_IMP_MAX 127  // Wait, 126 attempts to re-send
#define TWI_MSG_IMP_MASK 0x7F // b_0111_1111

#define SEND_STOP 1   // Send STOP signal
#define IGNORE_STOP 0 // STOP signal wouldn't be sent (REPEATED start expected)

// Transmit (RE)START->ADDR_WR->STOP message instead of STOP signal
#define TWI_MSG_STOP_BIT 7
#define TWI_MSG_STOP (1 << TWI_MSG_STOP_BIT)

#ifndef TWI_SOME_FREE_ADDR_WR
 #define TWI_SOME_FREE_ADDR_WR 238 // Any address that will not respond
#endif

//!!! Always add empty message before STOP signal
//#define TWI_FORCE_EMPTY_MSG_BEFORE_STOP

////////////////////////////////////////////////////////////////////////
// TWI State codes
// (raw TWSR value, assuming TWI prescaler bits/TWPS set to 0)
////////////////////////////////////////////////////////////////////////

// General TWI Master status codes
#define TWI_START                  0x08 // START has been transmitted
#define TWI_REP_START              0x10 // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38 // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK            0x18 // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20 // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK           0x28 // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30 // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK            0x40 // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48 // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50 // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK          0x58 // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0xA8 // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0 // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8 // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0 // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8 // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0x60 // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68 // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70 // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78 // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80 // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88 // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90 // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98 // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0 // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8 // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR              0x00 // Bus error due to an illegal START or STOP condition
