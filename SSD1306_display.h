#ifndef SSD1306_DISPLAY_H_INCLUDED
#define SSD1306_DISPLAY_H_INCLUDED

/*
 * Headers file for the controls for the OLED display based on SSD1306 driver.
 * Monochrome 128x64 or 128x32 dot matrix display.
 * Communications via I2C interface (TWI_Master.c file, M20230827).
 *
 * Version: M20230827
 */

#include "b_easy_bits.h"
#include "global_def.h"      // Global definitions (no variables init inside)


// If screen size is 128x32 the SSD1306_32 should be defined at compile time,
// see the global_def.h file.

#define SSD1306_EXT_01   // Add at compile time to extend default functions set
//#define SSD1306_EXT_02 // Add at compile time to extend default functions set
                         // Default:
                         //     SCR_set_contrast,
                         //     SCR_set_panel_power_on,
                         //     SCR_set_mem_mode,
                         //     SCR_set_col,
                         //     SCR_set_page,
                         //     SCR_set_hw_com_cfg,
                         //     SCR_clear,
                         //     SCR_clear_rect,
                         //     SCR_print_D,
                         //     SCR_wakeup.
                         // SSD1306_EXT_01:
                         //     SCR_set_all_dots_on,
                         //     SCR_set_inverse.
                         // SSD1306_EXT_02:
                         //     CR_set_col_L,
                         //     SCR_set_col_H,
                         //     SCR_set_page_L,
                         //     SCR_set_start_line
                         //     SCR_set_remap,
                         //     SCR_set_scan_direction,
                         //     SCR_set_v_offset,
                         //     SCR_set_freq.


////////////////////////////////////////////////////////////////////////
// Hardware properties
////////////////////////////////////////////////////////////////////////

// I2C address + r/w bit, whole byte
//#define SCR_ADDR_R  b_0111_1001 // Read mode, 0x79, NOT IN USE
//#define SCR_ADDR_WR b_0111_1000 // Write mode, 0x78
// I2C address + r/w, reserved doubles
#ifndef SCR_ADDR_R
 #define SCR_ADDR_R  b_0111_1011 // Read mode, 0x7B, NOT IN USE
#endif
#ifndef SCR_ADDR_WR
 #define SCR_ADDR_WR b_0111_1010 // Write mode, 0x7A
#endif

#define SCR_32MUX   0x1F // 32 lines
#define SCR_64MUX   0x3F // 64 lines

#ifdef SSD1306_32
 #define SCR_MUX_DEFAULT SCR_32MUX
#else
 #define SCR_MUX_DEFAULT SCR_64MUX
#endif

#define SCR_COMS (SCR_MUX_DEFAULT + 1)
#define SCR_SEGMENTS 128
#define SCR_DOTS_TOTAL (SCR_SEGMENTS * SCR_COMS)           // 8192 (64 lines)
#define SCR_1BYTE_DOTS 8
#define SCR_GDDRAM_BYTES (SCR_DOTS_TOTAL / SCR_1BYTE_DOTS) // 1024 (64 lines)
#define SCR_GDDRAM_PAGES (SCR_GDDRAM_BYTES / SCR_SEGMENTS) // 8 (64 lines)

#define SCR_MEM_HORIZ_MODE 0
#define SCR_MEM_VERT_MODE  1
#define SCR_MEM_PAGE_MODE  2

// NEXT configuration definitions says about HW layout of the matrix wires.
//
// In case of 64 lines the COM pins are interlaced on the OLED matrix,
// the wires from both sides of the glass goes to the bottom of the display:
// (row 3) L-----
//           -----R (row 2)
// (row 1) L-----
//           -----R (row 0)
//
// Thus, the SCR_CFG_INTERLACED will make sequential display of the GDDRAM
// content.
// The SCR_CFG_NORMAL will make interlaced display of the GDDRAM content.
// First bytes will be displayed on even rows (all R) and only then (rest of the GDDRAM)
// on each odd rows (all L).
//
// The 32 lines matrix has wires only on the right side of the glass,
// wires goes only to the right side of the display:
// -----R (row 3)
//  -----R (row 2)
// -----R (row 1)
//  -----R (row 0)
//
// Thus, the SCR_CFG_NORMAL will make sequential display of the GDDRAM
// content (all R rows).
// The 32 lines SSD1306 devices still may has GDDRAM of double size.

#define SCR_CFG_NORMAL             b_0000_0010
#define SCR_CFG_INTERLACED         b_0001_0010 // Common for 64 lines
#define SCR_CFG_NORMAL_FLIP_LR     b_0010_0010
#define SCR_CFG_INTERLACED_FLIP_LR b_0011_0010

// DCLK = F_OSC / D.
// F_FRAME =  F_OSC / (D * SCR_K * (SCR_MUX_DEFAULT + 1)).
// SCR_K = Phase 1 period + Phase 2 period + BANK0 pulse width.
#define SCR_F_OSC b_0000_1000 // Internal RC oscillator, 370 kHz
#define SCR_D     b_0000_0000 // Divide ratio, [0..15] + 1
#define PH1_T     2 // Phase 1, discharge [1..15] of DCLK
#define PH2_T     2 // Phase 2, charge [1..15] of DCLK
#define BANK0_PW 50 // Pulse width, segment current [??..50..??] of DCLK
#define SCR_K (PH1_T + PH2_T + BANK0_PW)
// Thus, default F_FRAME is ~107 Hz -> 0.00934 sec.
// I2C single frame = (ADDR + ACK) + (CMD + ACK) + N * (DATA + ACK) =
// = (8 + 1) + (8 + 1) + 128 * 64 / 8 * (8 + 1) = 9234 bits.
// I2C single frame = 9234 bits at 400 kHz -> 0.023085 sec.
// At defaults, the display will be updated ~2.47 times before full
// single frame can be transferred to GDDRAM via I2C (tearing).
// For 128x64 absolute max transfer ~43 fps (I2C), ideal conditions.


////////////////////////////////////////////////////////////////////////
// Master's logic
////////////////////////////////////////////////////////////////////////

// Control byte
#define CO_BIT 7
#define DC_BIT 6
// #define CONTROL_BYTE ((x << CO_BIT) | (y << DC_BIT))
//
// CO_BIT = 0 -> Multiple bytes comming next, then STOP
// CO_BIT = 1 -> Single byte comming next, then it requires
//               either STOP or new Control byte to be sent.
// DC_BIT = 0 -> Next is Command, or multibyte Command (as "Set Contrast")
// DC_BIT = 1 -> Next is Data, or multiple bytes of Data
#define SCR_MULTIBYTE_CMD  ((0 << CO_BIT) | (0 << DC_BIT)) // 0x00
#define SCR_MULTIBYTE_DATA ((0 << CO_BIT) | (1 << DC_BIT)) // 0x40
#define SCR_1BYTE_CMD      ((1 << CO_BIT) | (0 << DC_BIT)) // 0x80
#define SCR_1BYTE_DATA     ((1 << CO_BIT) | (1 << DC_BIT)) // 0xC0
// To avoid the bus overheat the multibyte command/data should come last
// in the transmission (to not send single byte command/data separetly).


////////////////////////////////////////////////////////////////////////
// Commands
////////////////////////////////////////////////////////////////////////

// 1. Fundamental
#define SCR_SET_CONTRAST 0x81 // Set contrast control
#define SCR_FROM_RAM_CMD b_1010_0100 // Enable RAM content display. Command = CMD | Value. Value = [0, 1].
#define SCR_DISPLAY_RAM  0xA4 // RAM content display
#define SCR_ALL_DOTS_ON  0xA5 // Entire display ON, ignore RAM content
#define SCR_INVERSE_CMD b_1010_0110 // Inverse RAM content display. Command = CMD | Value. Value = [0, 1].
#define SCR_NORMAL       0xA6 // Set normal display
#define SCR_INVERSE      0xA7 // Set inverse display
#define SCR_POWER_CMD b_1010_1110 // Power display. Command = CMD | Value. Value = [0, 1].
#define SCR_SLEEP        0xAE // Set display OFF, sleep operation mode
#define SCR_WAKE_UP      0xAF // Set Display ON, normal operation mode

// 2. Scrolling
#define SCR_SCROLL_R     0x26 // Continuous right horizontal scroll
#define SCR_SCROLL_L     0x27 // Continuous left horizontal scroll
#define SCR_SCROLL_VR    0x29 // Vertical and right horizontal scroll
#define SCR_SCROLL_VL    0x2A // Vertical and left horizontal scroll
#define SCR_SCROLL_STOP  0x2E // Deactivate scroll, display frozen
#define SCR_SCROLL_START 0x2F // Activate scroll
#define SCR_SCROLL_VAREA 0xA3 // Set vertical scroll area

// 3. Addressing setting
#define SCR_LOW_N_COL_CMD b_0000_0000 // Set column start address for page addressing mode (lower nibble). Command = CMD | Value. Value = [0..15].
#define SCR_LOW_N_COL_00 (SCR_LOW_N_COL_CMD | 0x00)
#define SCR_LOW_N_COL_01 (SCR_LOW_N_COL_CMD | 0x01)
#define SCR_LOW_N_COL_02 (SCR_LOW_N_COL_CMD | 0x02)
#define SCR_LOW_N_COL_03 (SCR_LOW_N_COL_CMD | 0x03)
#define SCR_LOW_N_COL_04 (SCR_LOW_N_COL_CMD | 0x04)
#define SCR_LOW_N_COL_05 (SCR_LOW_N_COL_CMD | 0x05)
#define SCR_LOW_N_COL_06 (SCR_LOW_N_COL_CMD | 0x06)
#define SCR_LOW_N_COL_07 (SCR_LOW_N_COL_CMD | 0x07)
#define SCR_LOW_N_COL_08 (SCR_LOW_N_COL_CMD | 0x08)
#define SCR_LOW_N_COL_09 (SCR_LOW_N_COL_CMD | 0x09)
#define SCR_LOW_N_COL_10 (SCR_LOW_N_COL_CMD | 0x0A)
#define SCR_LOW_N_COL_11 (SCR_LOW_N_COL_CMD | 0x0B)
#define SCR_LOW_N_COL_12 (SCR_LOW_N_COL_CMD | 0x0C)
#define SCR_LOW_N_COL_13 (SCR_LOW_N_COL_CMD | 0x0D)
#define SCR_LOW_N_COL_14 (SCR_LOW_N_COL_CMD | 0x0E)
#define SCR_LOW_N_COL_15 (SCR_LOW_N_COL_CMD | 0x0F)
#define SCR_HIGH_N_COL_CMD b_0001_0000 // Set column start address for page addressing mode (higher nibble). Command = CMD | Value. Value = [0..15].
#define SCR_HIGH_N_COL_16 (SCR_HIGH_N_COL_CMD | 0x00)
#define SCR_HIGH_N_COL_17 (SCR_HIGH_N_COL_CMD | 0x01)
#define SCR_HIGH_N_COL_18 (SCR_HIGH_N_COL_CMD | 0x02)
#define SCR_HIGH_N_COL_19 (SCR_HIGH_N_COL_CMD | 0x03)
#define SCR_HIGH_N_COL_20 (SCR_HIGH_N_COL_CMD | 0x04)
#define SCR_HIGH_N_COL_21 (SCR_HIGH_N_COL_CMD | 0x05)
#define SCR_HIGH_N_COL_22 (SCR_HIGH_N_COL_CMD | 0x06)
#define SCR_HIGH_N_COL_23 (SCR_HIGH_N_COL_CMD | 0x07)
#define SCR_HIGH_N_COL_24 (SCR_HIGH_N_COL_CMD | 0x08)
#define SCR_HIGH_N_COL_25 (SCR_HIGH_N_COL_CMD | 0x09)
#define SCR_HIGH_N_COL_26 (SCR_HIGH_N_COL_CMD | 0x0A)
#define SCR_HIGH_N_COL_27 (SCR_HIGH_N_COL_CMD | 0x0B)
#define SCR_HIGH_N_COL_28 (SCR_HIGH_N_COL_CMD | 0x0C)
#define SCR_HIGH_N_COL_29 (SCR_HIGH_N_COL_CMD | 0x0D)
#define SCR_HIGH_N_COL_30 (SCR_HIGH_N_COL_CMD | 0x0E)
#define SCR_HIGH_N_COL_31 (SCR_HIGH_N_COL_CMD | 0x0F)
#define SCR_MEM_MODE     0x20 // Set memory addressing mode
#define SCR_COLUMNS      0x21 // Setup column start and end address for horizontal or vertical addressing mode
#define SCR_PAGES        0x22 // Setup page start and end address for horizontal or vertical addressing mode
#define SCR_START_PAGE_CMD b_1011_0000 // Set GDDRAM page start address for page addressing mode. Command = CMD | Value. Value = [0..7].
#define SCR_START_PAGE0 (SCR_START_PAGE_CMD | 0x00)
#define SCR_START_PAGE1 (SCR_START_PAGE_CMD | 0x01)
#define SCR_START_PAGE2 (SCR_START_PAGE_CMD | 0x02)
#define SCR_START_PAGE3 (SCR_START_PAGE_CMD | 0x03)
#define SCR_START_PAGE4 (SCR_START_PAGE_CMD | 0x04)
#define SCR_START_PAGE5 (SCR_START_PAGE_CMD | 0x05)
#define SCR_START_PAGE6 (SCR_START_PAGE_CMD | 0x06)
#define SCR_START_PAGE7 (SCR_START_PAGE_CMD | 0x07)

#define SCR_START_PAGE_MASK b_0000_0111
#define SCR_END_PAGE_MASK   b_0000_0111

// 4. Hardware configuration (panel resolution & layout related)
#define SCR_START_LINE_CMD b_0100_0000 // Set display start line register. Command = CMD | Value. Value = [0..63].
#define SCR_START_LINE00 (SCR_START_LINE_CMD | 0x00)
#define SCR_START_LINE01 (SCR_START_LINE_CMD | 0x01)
#define SCR_START_LINE02 (SCR_START_LINE_CMD | 0x02)
#define SCR_START_LINE03 (SCR_START_LINE_CMD | 0x03)
#define SCR_START_LINE04 (SCR_START_LINE_CMD | 0x04)
#define SCR_START_LINE05 (SCR_START_LINE_CMD | 0x05)
#define SCR_START_LINE06 (SCR_START_LINE_CMD | 0x06)
#define SCR_START_LINE07 (SCR_START_LINE_CMD | 0x07)
#define SCR_START_LINE08 (SCR_START_LINE_CMD | 0x08)
#define SCR_START_LINE09 (SCR_START_LINE_CMD | 0x09)
#define SCR_START_LINE10 (SCR_START_LINE_CMD | 0x0A)
#define SCR_START_LINE11 (SCR_START_LINE_CMD | 0x0B)
#define SCR_START_LINE12 (SCR_START_LINE_CMD | 0x0C)
#define SCR_START_LINE13 (SCR_START_LINE_CMD | 0x0D)
#define SCR_START_LINE14 (SCR_START_LINE_CMD | 0x0E)
#define SCR_START_LINE15 (SCR_START_LINE_CMD | 0x0F)
#define SCR_START_LINE16 (SCR_START_LINE_CMD | 0x10)
#define SCR_START_LINE17 (SCR_START_LINE_CMD | 0x11)
#define SCR_START_LINE18 (SCR_START_LINE_CMD | 0x12)
#define SCR_START_LINE19 (SCR_START_LINE_CMD | 0x13)
#define SCR_START_LINE20 (SCR_START_LINE_CMD | 0x14)
#define SCR_START_LINE21 (SCR_START_LINE_CMD | 0x15)
#define SCR_START_LINE22 (SCR_START_LINE_CMD | 0x16)
#define SCR_START_LINE23 (SCR_START_LINE_CMD | 0x17)
#define SCR_START_LINE24 (SCR_START_LINE_CMD | 0x18)
#define SCR_START_LINE25 (SCR_START_LINE_CMD | 0x19)
#define SCR_START_LINE26 (SCR_START_LINE_CMD | 0x1A)
#define SCR_START_LINE27 (SCR_START_LINE_CMD | 0x1B)
#define SCR_START_LINE28 (SCR_START_LINE_CMD | 0x1C)
#define SCR_START_LINE29 (SCR_START_LINE_CMD | 0x1D)
#define SCR_START_LINE30 (SCR_START_LINE_CMD | 0x1E)
#define SCR_START_LINE31 (SCR_START_LINE_CMD | 0x1F)
#define SCR_START_LINE32 (SCR_START_LINE_CMD | 0x20)
#define SCR_START_LINE33 (SCR_START_LINE_CMD | 0x21)
#define SCR_START_LINE34 (SCR_START_LINE_CMD | 0x22)
#define SCR_START_LINE35 (SCR_START_LINE_CMD | 0x23)
#define SCR_START_LINE36 (SCR_START_LINE_CMD | 0x24)
#define SCR_START_LINE37 (SCR_START_LINE_CMD | 0x25)
#define SCR_START_LINE38 (SCR_START_LINE_CMD | 0x26)
#define SCR_START_LINE39 (SCR_START_LINE_CMD | 0x27)
#define SCR_START_LINE40 (SCR_START_LINE_CMD | 0x28)
#define SCR_START_LINE41 (SCR_START_LINE_CMD | 0x29)
#define SCR_START_LINE42 (SCR_START_LINE_CMD | 0x2A)
#define SCR_START_LINE43 (SCR_START_LINE_CMD | 0x2B)
#define SCR_START_LINE44 (SCR_START_LINE_CMD | 0x2C)
#define SCR_START_LINE45 (SCR_START_LINE_CMD | 0x2D)
#define SCR_START_LINE46 (SCR_START_LINE_CMD | 0x2E)
#define SCR_START_LINE47 (SCR_START_LINE_CMD | 0x2F)
#define SCR_START_LINE48 (SCR_START_LINE_CMD | 0x30)
#define SCR_START_LINE49 (SCR_START_LINE_CMD | 0x31)
#define SCR_START_LINE50 (SCR_START_LINE_CMD | 0x32)
#define SCR_START_LINE51 (SCR_START_LINE_CMD | 0x33)
#define SCR_START_LINE52 (SCR_START_LINE_CMD | 0x34)
#define SCR_START_LINE53 (SCR_START_LINE_CMD | 0x35)
#define SCR_START_LINE54 (SCR_START_LINE_CMD | 0x36)
#define SCR_START_LINE55 (SCR_START_LINE_CMD | 0x37)
#define SCR_START_LINE56 (SCR_START_LINE_CMD | 0x38)
#define SCR_START_LINE57 (SCR_START_LINE_CMD | 0x39)
#define SCR_START_LINE58 (SCR_START_LINE_CMD | 0x3A)
#define SCR_START_LINE59 (SCR_START_LINE_CMD | 0x3B)
#define SCR_START_LINE60 (SCR_START_LINE_CMD | 0x3C)
#define SCR_START_LINE61 (SCR_START_LINE_CMD | 0x3D)
#define SCR_START_LINE62 (SCR_START_LINE_CMD | 0x3E)
#define SCR_START_LINE63 (SCR_START_LINE_CMD | 0x3F)

#define SCR_MAP_SEG0_CMD b_1010_0000 // Re-mapping to SEG0. Command = CMD | Value. Value = [0, 1].
#define SCR_MAP_0_SEG0   0xA0 // Column address 0 is mapped to SEG0
#define SCR_MAP_127_SEG0 0xA1 // Column address 127 is mapped to SEG0
#define SCR_SET_MUX      0xA8 // Set multiplex ratio
#define SCR_SCAN_CMD     b_1100_0000 // Scan from COM0. Command = CMD | Value. Value = [0, 8].
#define SCR_SCAN_FWD     0xC0 // Scan from COM0 to COM[N – 1]
#define SCR_SCAN_BCKWRD  0xC8 // Scan from COM[N – 1] to COM0
#define SCR_COM_V_OFFSET 0xD3 // Set vertical shift by COM
#define SCR_COM_CONFIG   0xDA // Set COM pins hardware configuration

// 5. Timing & driving scheme setting
#define SCR_SET_CLOCKS   0xD5 // Set display clock divide ratio and oscillator frequency
#define SCR_PRECHARGE    0xD9 // Set pre-charge period
#define SCR_VCOMH_LEVEL  0xDB // Set VCOMH deselect level
#define SCR_NOP          0xE3 // Command for no operation

// 5a. Charge pump
#define SCR_CHRG_PUMP    0x8D // Charge pump setting
#define SCR_PUMP_EN  b_0001_0100 // Charge pump enabled
#define SCR_PUMP_DIS b_0001_0000 // Charge pump disabled


////////////////////////////////////////////////////////////////////////
// Functions declarations
////////////////////////////////////////////////////////////////////////

// Commands
void SCR_set_contrast(unsigned char);
void SCR_set_panel_power_on(unsigned char);
void SCR_set_mem_mode(unsigned char);
void SCR_set_col(unsigned char, unsigned char);
void SCR_set_page(unsigned char, unsigned char);
void SCR_set_hw_com_cfg(unsigned char);

// DATA operations
void SCR_clear_rect(unsigned char, unsigned char,
                    unsigned char, unsigned char);
unsigned char SCR_print_D (char* , unsigned char,
                           unsigned char, unsigned char,
                           unsigned char);

// General
void SCR_wakeup(void);

// Commands
#ifdef SSD1306_EXT_01
 void SCR_set_all_dots_on(unsigned char);
 void SCR_set_inverse(unsigned char);
#endif
#ifdef SSD1306_EXT_02
 void SCR_set_col_L(unsigned char);
 void SCR_set_col_H(unsigned char);
 void SCR_set_page_L(unsigned char);
 void SCR_set_start_line(unsigned char);
 void SCR_set_remap(unsigned char);
 void SCR_set_scan_direction(unsigned char);
 void SCR_set_v_offset(char);
 void SCR_set_freq(unsigned char);
#endif

#endif // SSD1306_DISPLAY_H_INCLUDED
