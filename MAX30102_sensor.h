#ifndef MAX30102_SENS_H_INCLUDED
#define MAX30102_SENS_H_INCLUDED

/*
 * Headers file for the controls for the optical sensor based on the MAX30102
 * IR, RED LED heart monitor and temperature sensor.
 * Communications via I2C interface (TWI_Master.c file, M20230827).
 *
 * Version: M20230827
 */

#include "b_easy_bits.h"     // Bit numbers to make coding easier
#include "global_def.h"      // Global definitions (no variables init inside)


//#define MAX30102_EXT_01 // Add at compile time to extend default functions set
//#define MAX30102_EXT_02 // Add at compile time to extend default functions set
//#define MAX30102_EXT_03 // Add at compile time to extend default functions set
                          // Default:
                          //     SENS_get_FIFO_pointers,
                          //     SENS_set_conversion,
                          //     SENS_set_LED_current,
                          //     SENS_get_rev_id,
                          //     SENS_get_FIFO_N_bytes,
                          //     SENS_get,
                          //     SENS_set.
                          // MAX30102_EXT_01:
                          //     SENS_set_die_tmpr_en,
                          //     SENS_get_die_tmpr_cfg,
                          //     SENS_get_die_tmpr_value.
                          // MAX30102_EXT_02:
                          //     SENS_get_status,
                          //     SENS_set_FIFO_pointers,
                          //     SENS_set_FIFO_cfg,
                          //     SENS_get_FIFO_cfg,
                          //     SENS_set_mode,
                          // MAX30102_EXT_03:
                          //     SENS_get_int_en,
                          //     SENS_set_int_en,



////////////////////////////////////////////////////////////////////////
// Hardware properties
////////////////////////////////////////////////////////////////////////

// I2C address + r/w, whole byte
#define SENS_ADDR_R  0xAF // Read mode
#define SENS_ADDR_WR 0xAE // Write mode


////////////////////////////////////////////////////////////////////////
// Command and data registers
////////////////////////////////////////////////////////////////////////

// Interrupts statistic
#define SENS_REG_INT_STAT1 0x00    // R
#define SENS_A_FULL_BIT    7       // for SENS_REG_INT_STAT1
#define SENS_PPG_RDY_BIT   6       // for SENS_REG_INT_STAT1
#define SENS_ALC_OVF_BIT   5       // for SENS_REG_INT_STAT1

#define SENS_REG_INT_STAT2    0x01 // R
#define SENS_DIE_TEMP_RDY_BIT 1    // for SENS_REG_INT_STAT2

#define SENS_REG_INT_ENBL1  0x02   // RW
#define SENS_A_FULL_EN_BIT  7      // for SENS_REG_INT_ENBL1
#define SENS_PPG_RDY_EN_BIT 6      // for SENS_REG_INT_ENBL1
#define SENS_ALC_OVF_EN_BIT 5      // for SENS_REG_INT_ENBL1

#define SENS_REG_INT_ENBL2       0x03 // RW
#define SENS_DIE_TEMP_RDY_EN_BIT 1    // for SENS_REG_INT_ENBL2

// FIFO buffer properties
#define SENS_REG_FIFO_WR_PTR  0x04 // RW
#define SENS_FIFO_WR_PTR_MASK b_0001_1111

#define SENS_REG_OVF_COUNTER  0x05 // RW
#define SENS_OVF_COUNTER_MASK b_0001_1111

#define SENS_REG_FIFO_RD_PTR  0x06 // RW
#define SENS_FIFO_RD_PTR_MASK b_0001_1111

#define SENS_REG_FIFO_DATA    0x07 // RW

#define SENS_REG_FIFO_CFG     0x08    // RW
#define SENS_SMP_AVE_1    b_0000_0000 // no averaging
#define SENS_SMP_AVE_2    b_0010_0000
#define SENS_SMP_AVE_4    b_0100_0000
#define SENS_SMP_AVE_8    b_0110_0000
#define SENS_SMP_AVE_16   b_1000_0000
#define SENS_SMP_AVE_32   b_1010_0000
#define SENS_SMP_AVE_MASK b_1110_0000 // for SENS_REG_FIFO_CFG
#define SENS_FIFO_ROLLOVER_EN_BIT 4   // for SENS_REG_FIFO_CFG
#define SENS_FIFO_A_FULL_MASK b_0000_1111 // for SENS_REG_FIFO_CFG.
                                          // Empty space (number of empty data
                                          // samples) in FIFO when interrupt is
                                          // issued.

// Max sensor operation mode
#define SENS_REG_MODE_CFG 0x09     // RW
#define SENS_SHDN_BIT  7           // for SENS_REG_MODE_CFG
#define SENS_RESET_BIT 6           // for SENS_REG_MODE_CFG
//!!! The LED1 is IR (for some chinese clones/revisions)
// The smartphones with cheaper front camera should be used to check the LED.
// Visual confirmation. Samsung Galaxy A01 smartphone, IR LED current:
//  front camera >  2 mA (barely visible)
//  rear camera  > 12 mA (barely visible), has better IR filter!
#define SENS_MODE_DEFAULT    b_0000_0000 // After reset condition
#define SENS_MODE_HEART_RATE b_0000_0010 // LED1 (IR, rev = 0x03, id = 0x15)
#define SENS_MODE_SPO2       b_0000_0011 // LED1 + LED2
#define SENS_MODE_MULTI_LED  b_0000_0111 // Custom
#define SENS_MODE_MASK       b_0000_0111 // for SENS_REG_MODE_CFG

// Signal/conversion settings
#define SENS_REG_SPO2_CFG 0x0A         // RW
// ADC range, 18-bit resolution (262144 steps).
// If light is strong then the scale should be increased to avoid saturation.
// For SENS_REG_SPO2_CFG.
#define SENS_SPO2_ADC_RGE_2048  b_0000_0000 // Full scale = 2048 nA
#define SENS_SPO2_ADC_RGE_4096  b_0010_0000 // Full scale = 4096 nA
#define SENS_SPO2_ADC_RGE_8192  b_0100_0000 // Full scale = 8192 nA
#define SENS_SPO2_ADC_RGE_16384 b_0110_0000 // Full scale = 16384 nA
#define SENS_SPO2_ADC_RGE_MASK  b_0110_0000 // for SENS_REG_SPO2_CFG
#define SENS_SPO2_ADC_RGE_BITS  5 // Bit position for LSB of the range bits in the SENS_REG_SPO2_CFG register
// Sample rate, 1 sample = 1 Red conversion + 1 IR conversion (6 bytes)
// This setting has less priority and limited by the LED_PW setting.
#define SENS_SPO2_SR_50   b_0000_0000 // 50 samples/s
#define SENS_SPO2_SR_100  b_0000_0100 // 100 samples/s
#define SENS_SPO2_SR_200  b_0000_1000 // 200 samples/s
#define SENS_SPO2_SR_400  b_0000_1100 // 400 samples/s
#define SENS_SPO2_SR_800  b_0001_0000 // 800 samples/s
#define SENS_SPO2_SR_1000 b_0001_0100 // 1000 samples/s
#define SENS_SPO2_SR_1600 b_0001_1000 // 1600 samples/s
#define SENS_SPO2_SR_3200 b_0001_1100 // 3200 samples/s
#define SENS_SPO2_SR_MASK b_0001_1100 // for SENS_REG_SPO2_CFG
// LED pulse width (energy).
// Longer pulse -> longer time for ADC conversion allowed.
// Longer time for ADC conversion -> ADC resolution (bitness) can be increased.
#define SENS_SPO2_LED_PW_15MAX b_0000_0000 // 15 bit, takes 69 (68.95) us
#define SENS_SPO2_LED_PW_16MAX b_0000_0001 // 16 bit, takes 118 (117.78) us
#define SENS_SPO2_LED_PW_17MAX b_0000_0010 // 17 bit, takes 215 (215.44) us
#define SENS_SPO2_LED_PW_18MAX b_0000_0011 // 18 bit, takes 411 (410.75) us
#define SENS_SPO2_LED_PW_MASK  b_0000_0011 // for SENS_REG_SPO2_CFG

// LED current (energy)
#define SENS_REG_LED1_PA 0x0C // RW
                              // LED1 pulse amplitude = Value * 0.2 mA
                              // IR (chinese clone),  rev = 0x03, id = 0x15
#define SENS_REG_LED2_PA 0x0D // RW
                              // LED2 pulse amplitude = Value * 0.2 mA
                              // Red (chinese clone),  rev = 0x03, id = 0x15

// Custom LED interleaving (Multi-LED mode).
// One SLOT = 3 bytes of ADC data.
// One sample = SLOT1 + SLOT2 (when both enabled).
// Number of enabled SLOTs determines the sample length.
#define SENS_REG_MULTI_LED_SLOT12_CFG 0x11 // RW

#define SENS_REG_MULTI_LED_SLOT34_CFG 0x12 // RW

#define SENS_SLOT_DIS   b_0000_0000 // Off; no ADC conversion
#define SENS_SLOT_LED1  b_0000_0001 // IR (chinese clone),  rev = 0x03, id = 0x15
#define SENS_SLOT_LED2  b_0000_0010 // Red (chinese clone), rev = 0x03, id = 0x15
#define SENS_SLOT_SKIP1 b_0000_0011 // Off; ambient light ADC?
#define SENS_SLOT_SKIP2 b_0000_0100 // Off; ambient light ADC?
#define SENS_SLOT1_LS 0 // Left shift for the SENS_SLOT_...
#define SENS_SLOT2_LS 4 // Left shift for the SENS_SLOT_...
#define SENS_SLOT3_LS 0 // Left shift for the SENS_SLOT_...
#define SENS_SLOT4_LS 4 // Left shift for the SENS_SLOT_...

// For SENS_REG_MULTI_LED_SLOT12_CFG
#define SENS_SLOT1_DIS   (SENS_SLOT_DIS   << SENS_SLOT1_LS)
#define SENS_SLOT1_LED1  (SENS_SLOT_LED1  << SENS_SLOT1_LS)
#define SENS_SLOT1_LED2  (SENS_SLOT_LED2  << SENS_SLOT1_LS)
#define SENS_SLOT1_SKIP  (SENS_SLOT_SKIP2 << SENS_SLOT1_LS)

#define SENS_SLOT2_DIS   (SENS_SLOT_DIS   << SENS_SLOT2_LS)
#define SENS_SLOT2_LED1  (SENS_SLOT_LED1  << SENS_SLOT2_LS)
#define SENS_SLOT2_LED2  (SENS_SLOT_LED2  << SENS_SLOT2_LS)
#define SENS_SLOT2_SKIP  (SENS_SLOT_SKIP2 << SENS_SLOT2_LS)

// For SENS_REG_MULTI_LED_SLOT34_CFG
#define SENS_SLOT3_DIS   (SENS_SLOT_DIS   << SENS_SLOT3_LS)
#define SENS_SLOT3_LED1  (SENS_SLOT_LED1  << SENS_SLOT3_LS)
#define SENS_SLOT3_LED2  (SENS_SLOT_LED2  << SENS_SLOT3_LS)
#define SENS_SLOT3_SKIP  (SENS_SLOT_SKIP2 << SENS_SLOT3_LS)

#define SENS_SLOT4_DIS   (SENS_SLOT_DIS   << SENS_SLOT4_LS)
#define SENS_SLOT4_LED1  (SENS_SLOT_LED1  << SENS_SLOT4_LS)
#define SENS_SLOT4_LED2  (SENS_SLOT_LED2  << SENS_SLOT4_LS)
#define SENS_SLOT4_SKIP  (SENS_SLOT_SKIP2 << SENS_SLOT4_LS)

// For SENS_REG_MULTI_LED_SLOT12_CFG
#define SENS_MULTI_LED_SLOT1_MASK b_0000_0111
#define SENS_MULTI_LED_SLOT2_MASK b_0111_0000

// For SENS_REG_MULTI_LED_SLOT34_CFG
#define SENS_MULTI_LED_SLOT3_MASK b_0000_0111
#define SENS_MULTI_LED_SLOT4_MASK b_0111_0000

// MAX sensor temperature measurement
#define SENS_REG_DIE_TMPR_INT_VAL  0x1F // R

#define SENS_REG_DIE_TMPR_FRAC_VAL 0x20 // R
#define SENS_TFRAC_MASK b_0000_1111     // for SENS_REG_DIE_TMPR_FRAC_VAL

#define SENS_REG_DIE_TMPR_CFG 0x21      // RW
#define SENS_TMPR_EN_BIT 0              // for SENS_REG_DIE_TMPR_CFG

// Temperature ADC acquisition time, in MCU cycles
#define SENS_TT_CYCL ((F_CPU * 29) / 1000 + 1) // 29 ms

// MAX sensor revision and ID
#define SENS_REG_REV_ID  0xFE // R

#define SENS_REG_PART_ID 0xFF // R
#define MAX30102_PART_ID 0x15

// Conventional constants and definitions
#define SENS_SMPL_SIZE 3 // Size of the single LED sample in the FIFO, in bytes
#define SENS_SMPL_LNGTH_HR   SENS_SMPL_SIZE       // LED1, sample length
#define SENS_SMPL_LNGTH_SpO2 (2 * SENS_SMPL_SIZE) // LED1 + LED2

// Chinese clone I have, default register values:
// 0x08 = 0x0F
// 0x13 = 0x0F
// 0x14-0x17 = 0xFF
// Everything else in range 0x00-0x2F is 0x00
//
// 0xFE = 0x03
// 0xFF = 0x15

////////////////////////////////////////////////////////////////////////
// Functions declarations
////////////////////////////////////////////////////////////////////////

// Status
#ifdef MAX30102_EXT_02
unsigned char SENS_get_status(void);
#endif

#ifdef MAX30102_EXT_02
unsigned char SENS_get_int_en(void);
void SENS_set_int_en(void);
#endif

// FIFO
unsigned char SENS_get_FIFO_pointers(void);

#ifdef MAX30102_EXT_02
void SENS_set_FIFO_pointers(void);

// Configuration
void SENS_set_FIFO_cfg(void);
unsigned char SENS_get_FIFO_cfg(void);
void SENS_set_mode(void);
#endif

void SENS_set_conversion(void);
void SENS_set_LED_current(void);

// Die Temperature
#ifdef MAX30102_EXT_01
void SENS_set_die_tmpr_en(void);
unsigned char SENS_get_die_tmpr_cfg(void);
unsigned char SENS_get_die_tmpr_value(void);
#endif

// Part ID
unsigned char SENS_get_rev_id(void);

// DATA operations
unsigned char SENS_get_FIFO_N_bytes(void);

// Service
unsigned char SENS_get(void);
void SENS_set(void);

#endif // MAX30102_SENS_H_INCLUDED
