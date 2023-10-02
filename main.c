/*
 * The MCU Clock Speed in this project is set to 8 000 000 Hz
 * MCU model: ATmega88PA
 *
 * Eternal devices:
 *  - SSD1306 Graphic OLED (128x64, monochrome, 1-bit)
 *  - MAX30102 Pulse Oximeter and Heart-Rate sensor (2 LEDs)
 */

#include <avr/io.h>       // MCU Registers
#include <avr/pgmspace.h> // Do not load const to RAM
#include <stdio.h>        // FILE, puts() etc
#include <avr/interrupt.h>
#include <avr/wdt.h>      // Disable watchdog
#include <avr/builtins.h> // Delay in cycles
#include <stdfix.h>       // Fixed point for R calculations
#include <avr/sleep.h>

#include "TWI_Master.h"           // I2C driver
#include "SSD1306_display.h"      // Display driver
#include "usart_for_debug.h"      // Messages via UART
#include "short_uint_to_string.h" // Integer to string
#include "pins_config.h"          // Physical connections of the buttons/LEDs etc
#include "global_def.h"           // Global definitions
#include "MAX30102_sensor.h"      // MAX sensor
#include "MAX30102_approx.h"

extern FILE myout; // usart_for_debug.c

// Array to store decimal/string for display or debug messages
//
// Limitations:
//   hex max storage -> FFFF (4 characters + '\0')
//   unsigned short int max storage -> 65535 (5 characters + '\0')
// The last position ([6]) is fixed and stores total number of digits.
char disp_str[DIS_STR];

// Just simple const string that appears in the code many times
const static char LF[] PROGMEM = "\n";

// Time for power to stabilize after powering up external hardware
#define POWERSTAB_PAUSE_MSEC 80 // ~80 ms

// Shorten notation for "Sample size, in bytes"
#define S_SIZE SENS_SMPL_SIZE

// Buffer to send data/commands to the display device
unsigned char screen_buff[TWI_BUFFER_SIZE] = {SCR_ADDR_WR}; // Filled with address

// Storage for data from the sensor, also used as buffer to send commands to the sensor
unsigned char sens_buff[TWI_BUFFER_SIZE];

// Common byte definitions during data exchange with the sensor
#define INT_STAT1 sens_buff[1]
#define INT_STAT2 sens_buff[2]
#define INT_ENBL1 sens_buff[1]
#define INT_ENBL2 sens_buff[2]
#define FIFO_WR_PTR sens_buff[1]
#define OVF_COUNTER sens_buff[2]
#define FIFO_RD_PTR sens_buff[3]
#define FIFO_CFG sens_buff[1]
#define MODE_CFG sens_buff[1]
#define SPO2_CFG sens_buff[1]
#define LED1_PA sens_buff[1]
#define LED2_PA sens_buff[2]
#define TINT  sens_buff[1]
#define TFRAC sens_buff[2]
#define TMPR_CFG sens_buff[1]
#define TMPR_EN (TMPR_CFG & (1 << SENS_TMPR_EN_BIT))
#define NOT_TMPR_EN ((~TMPR_CFG) & (1 << SENS_TMPR_EN_BIT))
#define REV_ID  sens_buff[1]
#define PART_ID sens_buff[2]
#define N_BYTES sens_buff[1]

static unsigned char sens_mode = SENS_MODE_SPO2; // Sensor mode: HR, SpO2, Multi-LED etc.

static unsigned char fs_val = SENS_SPO2_ADC_RGE_2048; // Full scale = 2048 nA

#define SR_SET_VALUE SENS_SPO2_SR_50 // Recommended upper limit is 400 samples/s, see calc_heart_rate()
#define SEC_IN_1MINUTE 60

#if (SR_SET_VALUE == SENS_SPO2_SR_50)
 #define SR_VALUE_SAMPLES 50
#elif (SR_SET_VALUE == SENS_SPO2_SR_100)
 #define SR_VALUE_SAMPLES 100
#elif (SR_SET_VALUE == SENS_SPO2_SR_200)
 #define SR_VALUE_SAMPLES 200
#elif (SR_SET_VALUE == SENS_SPO2_SR_400)
 #define SR_VALUE_SAMPLES 400
#elif (SR_SET_VALUE == SENS_SPO2_SR_800)
 #define SR_VALUE_SAMPLES 800
#elif (SR_SET_VALUE == SENS_SPO2_SR_1000)
 #define SR_VALUE_SAMPLES 1000
#elif (SR_SET_VALUE == SENS_SPO2_SR_1600)
 #define SR_VALUE_SAMPLES 1600
#else
 #define SR_VALUE_SAMPLES 3200
#endif

// Sane interval ranges for single byte (255..1 beats/min) heart rates:
//   50 --> [12..3000]
//  100 --> [24..6000]
//  200 --> [48..12000]
//  400 --> [95..24000]
//  800 --> [189..48000]
// 1000 --> [236..60000]
// 1600 --> [377..96000]
// 3200 --> [753..192000]
#define SANE_INTERVAL_MIN (SR_VALUE_SAMPLES * SEC_IN_1MINUTE / UINT8_MAX + 1)

#if (SR_VALUE_SAMPLES * SEC_IN_1MINUTE > UINT16_MAX)
 #error "Too much samples per second to fit 1 minute of them into UINT16"
#endif

// LED current. 1 LSB = 0.2 mA
const static unsigned char curr_per_range[4] = {0x1F, 0x37, 0x67, 0xAF}; // 6.2|11|20|35 mA
static unsigned char LED1_current = 0x1F; // 6.2 mA
static unsigned char LED2_current = 0x1F; // 6.2 mA

// These variables modified inside/outside interrupts
volatile static unsigned char debounce_complete = 0; // Debouncing timer/pins_state
volatile static unsigned char holds_counter[] = {0, 0, 0}; // [0] - reserved
volatile static unsigned char bt1 = 0; // Default Button 1 state is not pressed
volatile static unsigned char bt2 = 0; // Default Button 2 state is not pressed

// 1 msec in MCU cycles
#define ONE_MSEC_CYCL (F_CPU / 1000)

// Setting defaults and update required fields
static void sens_reset_to_HR_SpO2(unsigned char mode) {
    sens_buff[0] = 1;
    sens_buff[1] = (1 << SENS_RESET_BIT); // MODE_CFG
    sens_buff[1 + 1] = SENS_REG_MODE_CFG;
    SENS_set();

    // Should wait until reset done before sending new command
    //__builtin_avr_delay_cycles(1 * ONE_MSEC_CYCL); // ~1 msec

    // Set many parameters in one transfer
    sens_buff[0] = 5;
    sens_buff[1] = mode; // MODE_CFG
    sens_buff[2] = fs_val | SR_SET_VALUE | SENS_SPO2_LED_PW_16MAX; // SPO2_CFG
    sens_buff[3] = 0; // RESERVED
    sens_buff[4] = LED1_current; // LED1_PA
    sens_buff[5] = LED2_current; // LED2_PA
    sens_buff[5 + 1] = SENS_REG_MODE_CFG;
    SENS_set();
};

// Button's max hold time, in milliseconds
#define HOLD_THRESHOLD_MSEC 1000
#define HOLD_PWR_THRESHOLD_MSEC (HOLD_THRESHOLD_MSEC + 2000)
// btn_state top
#define ABS_MAX_HOLD 255

// Get state of the button.
// ONLY call it in timed intervals to get suitable hold time.
// The hold time increments only in this function call.
// Returns: 0 - not pressed; > 0 - pressed;
// ---------------------------------------------------------------------
// btn_msk = 0..255 - the mask for the pins of the port, that assumed to be a buttons
// cell = 0..255 - the cell number that stores holds_counter for the button
static unsigned char get_button_state(const unsigned char btn_msk,
                               const unsigned char cell) {
    unsigned char btn_state = 0;

    // Re-use variable, so temporary debounce_complete = pins_state
    if (debounce_complete & btn_msk) {
        // Pin is on high level after debounce ended,
        // thus the button is not pressed.
        // Reset timer's counter (prepare to the next hold).
        holds_counter[cell] = 0;
    } else {
        // Pin is on low level after debounce,
        // the button pressed and hold.
        // The holds counter itself shouldn't overflow.
        if (holds_counter[cell] < ABS_MAX_HOLD) {
            ++holds_counter[cell];
        }

        btn_state = holds_counter[cell]; // The holds count
    }
    return btn_state;
}

// Give the power to the external devices and wait for voltage to stabilize
// Low signal at the pin powers up external devices
static void power_up_ext_hw(void) {
    // Intermediate output high from pulled-up (set DDR, Port = 1)
    // or instant output low (set DDR, Port = 0)
    DEV_POWER_DDR |= ALL_DEV_POWER_MSK;
    // Ensure output low (clear PORT, DDR = 1)
    DEV_POWER_PORT &= ~ALL_DEV_POWER_MSK;

    // Wait for voltage to stabilize
    __builtin_avr_delay_cycles(POWERSTAB_PAUSE_MSEC * ONE_MSEC_CYCL);
}

// Shut down the power from the external devices
// High (pulled-up) signal at the pin powers down external devices
static void power_down_ext_hw(void) {
    // Intermediate output high from output low (set PORT, DDR = 1)
    // or instant input pull-up (set PORT, DDR = 0)
    DEV_POWER_PORT |= ALL_DEV_POWER_MSK;
    // Ensure input pull-up (clear DDR, Port = 1)
    DEV_POWER_DDR &= ~ALL_DEV_POWER_MSK;
}

// List all connected I2C devices
static void scan_twi(void) {
    fputs_P(PSTR("I2C scan...\n"), stdout);
    fputs_P(PSTR("devices:\n"), stdout);

    // For up to 400kHz transmissions
    // available decimal range of "address+r/w" is [16; 239].
    // Look only for devices enabled for WRITE.
    for (unsigned char guess = 16; guess < 240; guess += 2) {
        TWI_Start_Transceiver_With_Data(&guess, 1, TWI_MSG_IMP_LOW);

        // Wait for transfer to complete.
        // If device responded - print address to UART.
        if (TWI_Complete_Last_Transfer(IGNORE_STOP)) {
            fputs_P(PSTR(" 0x"), stdout);
            fputs(usitohex(&disp_str[0], (unsigned short int)guess), stdout);
            fputs_P(LF, stdout);
        }
    }

    // SCL remains low, waiting for the next STOP or REPEATED start,
    // try STOP with the timeout.
    TWI_STOP_Signal();
    fputs_P(PSTR("scan complete\n"), stdout);
}

// Convert FIFO data from sensor to usable format
static void bytedata24be_to_16le(const unsigned char i) {
    // Build 16-bit Little-endian numbers from
    // the 24-bit Big-endian byte data (it has very special left-justified 18-bit storage).
    // No swap, but use extra cell for conversion.
    //
    // Before:
    //    buffer[x, b1, b2, b3, b1, b2, b3, ...]
    //              |sample1 |  |sample2 |
    //              |  LED1  |  |  LED1  |
    //
    // After:
    //    buffer[x, low1, high1, low1, low2, high2, low2, ...]
    //              | sample1 |        | sample2 |
    //              |   LED1  |        |   LED1  |

    // Low byte
    sens_buff[i * S_SIZE] >>= 2;
    sens_buff[i * S_SIZE] |= sens_buff[i * S_SIZE - 1] << 6;

    // High byte
    sens_buff[i * S_SIZE - 1] >>= 2;
    sens_buff[i * S_SIZE - 1] |= sens_buff[i * S_SIZE - 2] << 6;

    // Copy Low before High
    sens_buff[i * S_SIZE - 2] = sens_buff[i * S_SIZE];

    // Now the 16le value = *((unsigned short int*)&sens_buff[i * S_SIZE - 2])
}

// Window width for the peak detector
#define PEAK_WND_WIDTH 9
#if (PEAK_WND_WIDTH > SANE_INTERVAL_MIN)
 #warning "PEAK_WND_WIDTH too high, possible max detectable frequency will be lower than expected"
#endif

// Look for the extremum in the window and return position of the extremum
static unsigned char find_extremum_pos(unsigned char find_min, unsigned char current_pos,
                                unsigned short int dist, unsigned short int smpl[]) {
    unsigned char extremum_pos = current_pos;
    unsigned short int extremum = smpl[current_pos];

    // Limit the window to run algorithm from current position to previous extremum
    unsigned char window_width = PEAK_WND_WIDTH;
    if (dist < PEAK_WND_WIDTH) {
        window_width = (unsigned char) dist;
    }

    // Cycle backward through the elements of the window
    while (window_width) {
        if (current_pos == 0) {
            current_pos = PEAK_WND_WIDTH;
        }
        --current_pos;
        --window_width;

        if (find_min) {
            if (smpl[current_pos] < extremum) {
                extremum = smpl[current_pos];
                extremum_pos = current_pos;
            }
        } else {
            if (smpl[current_pos] > extremum) {
                extremum = smpl[current_pos];
                extremum_pos = current_pos;
            }
        }
    }

    return extremum_pos;
}

// Restore extremum's value from current sample value and previous differences.
// Return distance from the current position to the restored extremum position.
static unsigned char restore_smpl_value(unsigned short int* smpl_p, short int diff[],
                                        unsigned char extremum_pos, unsigned char current_pos) {
    unsigned char n_smpls_ago = 0;
    while (current_pos != extremum_pos) {
        *smpl_p -= diff[current_pos]; //! unsigned -= signed

        // Extremum lies N-samples ago
        ++n_smpls_ago;

        // Cycle backward through the elements of the window.
        if (current_pos == 0) {
            current_pos = PEAK_WND_WIDTH;
        }
        --current_pos;
    }
    return n_smpls_ago;
}

// Calculate DC
// Here "min_old_p" stores old minimum that will be overwritten with actual DC value
static void calc_dc(unsigned short int* min_old_p, unsigned short int min,
                    unsigned short int min_old_to_max_dist, unsigned short int current_dist) {
    if (*min_old_p > min) {
        *min_old_p -= min; // min_old - min
        *min_old_p *= current_dist;
        *min_old_p /= (min_old_to_max_dist + current_dist);
        *min_old_p += min; // DC lies above current min
    } else {
        *min_old_p = min - *min_old_p;
        *min_old_p *= current_dist;
        *min_old_p /= (min_old_to_max_dist + current_dist);
        *min_old_p = min - *min_old_p; // DC lies below current min
    }
}

// Get decimal from fraction, 2 digits
static unsigned char dec_from_frac(unsigned accum r) {
#define FRAC_DEC_DIV_2DIGITS 655 // Divider = (2^UACCUM_FBIT) / 100 = 655.36
    unsigned short int r_fraction;
    r_fraction = bitsuk(r);
    r_fraction /= FRAC_DEC_DIV_2DIGITS;

#define R_DECIMAL_MAX 99 // Maximum for 2 digits (0.99)
    unsigned char r_decimal;
    r_decimal = (unsigned char)r_fraction;
    if (r_decimal > R_DECIMAL_MAX) {
        r_decimal = R_DECIMAL_MAX;
    }
    return r_decimal;
}

// Calculate R, display SpO2(%)
// R = (AC_red / DC_red) / (AC_ir / DC_ir)
static void calc_r(unsigned short int ac_ir, unsigned short int dc_ir,
             unsigned short int ac_red, unsigned short int dc_red) {
    // Fixed point calculations ("unsigned accum" type) ISO/IEC TR 18037 + AVR:
    //   UACCUM_IBIT = 16
    //   UACCUM_FBIT = 16
    //   UACCUM_MIN = 0x00000000
    //   UACCUM_MAX = 0xFFFFFFFF
    //   UACCUM_EPSILON = 0x00000001

    // Prevent division by zero (avoid NaN results)
    if (ac_red & dc_red & ac_ir & dc_ir) {
        // Calculate 1/R.
        // Use only divisions to reduce code size for fixed point math.
        unsigned accum reciprocal_r;
        reciprocal_r = ((unsigned accum)ac_ir / (unsigned accum)dc_ir) / ((unsigned accum)ac_red / (unsigned accum)dc_red);

        unsigned char last_drawpoint = 65;

        // Display integer part of 1/R
        disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)reciprocal_r, 0);
        last_drawpoint += SCR_print_D(&disp_str[0], FONT_8PX, last_drawpoint, last_drawpoint + (3+1)*5-1, 1 | ALIGN_LEFT);

        disp_str[0] = 1; // "Decimal point"
        disp_str[DIS_STR_NOD] = 1;
        last_drawpoint += SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, last_drawpoint + (1+1)*1-1, 1 | ALIGN_LEFT);

        // Display decimal part of 1/R, 2 digits
        // In the end clear maximum possible space that shared between
        // drawings of 1/R (30 px) and temperature (30 px).
        disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)dec_from_frac(reciprocal_r), 0);
        SCR_print_D(&disp_str[0], FONT_8PX, last_drawpoint, 65 + (4*5+2+4*2)-1, 1 | ALIGN_LEFT);

        // Prevent division by zero (avoid NaN results)
        if (reciprocal_r) {
            // Calculate SpO2(%) from approximated line
            unsigned char SpO2;
            SpO2 = (unsigned char)(K_APPROX_SpO2 / reciprocal_r); //!!! "+"

            // Clamp to bounds of SpO2 approximation equation
            if (SpO2 < TOP_APPROX) {
                SpO2 = TOP_APPROX;
            } else if (SpO2 > BOTTOM_APPROX) {
                SpO2 = BOTTOM_APPROX; // Zero value can be forced with B_APPROX_SpO2 here
            }

            SpO2 = B_APPROX_SpO2 - SpO2; //!!! "-"

            // Display SpO2(%), always 2 digits
            disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)SpO2, 0);
            SCR_print_D(&disp_str[0], FONT_32PX, 0, 0+(18+3)*2-1, 4 | ALIGN_RIGHT);
            disp_str[0] = 2; // "Percent"
            disp_str[DIS_STR_NOD] = 1;
            SCR_print_D(&disp_str[0], IMGS_24PX, 0+(18+3)*2-1+1+1, 0+(18+3)*2-1+1+1+20-1, 5 | ALIGN_LEFT);
        }

        // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
        TWI_Complete_Last_Transfer(SEND_STOP);
    }
}

// Calculate heart rate (beats/min)
static unsigned char calc_heart_rate(unsigned short int interval) {
    unsigned char heart_rate;

    // Heart rate calculation with saturation, also prevents division by zero
    if (interval < SANE_INTERVAL_MIN) {
        heart_rate = UINT8_MAX;
    } else {
        heart_rate = SR_VALUE_SAMPLES * SEC_IN_1MINUTE / interval;
    }
    return heart_rate;
}

// Area for photoplethysmogram
//
// MAX - - - -
// y   - - - -
// y   - - - -
// 0   - - - -
//     0 x x MAX
#define PPG_XMIN 0
#define PPG_XMAX 63
#define PPG_YMIN 0
#define PPG_YMAX 31

#define PPG_EDGE_DIV 3 // Divider to increase dynamic range of the displayed signal
                       // to better fit spikes into display area (+33% at top and
                       // -33% at bottom).

// Column byte structure.
// The dot shown at y = MAX. Can be implemented as left shift = 0 of the absolute maximum.
//
//             0yyy yyyy yyyy yyyy yyyy yyyy yyyy yyyMAX
//             0000_0000 0000_0000 0000_0000 0000_0001
//    RAM byte    [3]       [2]       [1]       [0]
//    OLED row                         ...98 7654 3210
//  Perception  bottom__________________________top
#define DOT_AT_MAX b_0000_0001 // [0]

// Height of photoplethysmogram in bytes
#define BYTES_IN_PPG_COLUMN ((PPG_YMAX + 1) / SCR_1BYTE_DOTS)

//#define PPG_RAW

// Render photoplethysmogram (PPG):
//  1) raw; not absorbed light in graph is highlighted, blood rich areas dimmed (systolic at min)
//
//              Systole
//                  | Diastole
//              +   |   |   +           +
//            ++++  |   | ++++        ++++
//        +++++++++ | +++++++++   +++++++++
//     + +++++++++++ +++++++++++ +++++++++++
//
//  2) normal; not absorbed light in graph is dimmed, blood rich areas highlighted (systolic at max)
//
//              Systole
//                  | Diastole
//      +           +   |       +
//     +++         +++  |      +++         +
//     +++++++    ++++++++    ++++++++    ++
//     +++++++++ +++++++++++ +++++++++++ +++

static void render_ppg(const unsigned short int smpl, const unsigned short int ac,
                        const unsigned short int dc, const unsigned char ppg_pos) {
    const unsigned short int signal_room = ac / PPG_EDGE_DIV;
    const unsigned short int smpl_d_max = ac + signal_room * 2; // Increase possible display maximum
    const unsigned short int smpl_ac = smpl - (dc - signal_room); // Reduce DC signal to shift displayed value up

#ifdef PPG_RAW
    unsigned char shift = 0;
#else
    unsigned char shift = PPG_YMAX;
#endif

    // Prevent division by zero and overflow
    if (smpl_d_max && (smpl_ac < smpl_d_max)) {
#ifdef PPG_RAW
        shift = (unsigned char)(PPG_YMAX - smpl_ac * PPG_YMAX / smpl_d_max); // Left shift, total
#else
        shift = (unsigned char)(smpl_ac * PPG_YMAX / smpl_d_max); // Left shift, total
#endif
    }

#define NUM_OF_UPD_COLUMNS 2 // 1st - data, 2nd - clear one column ahead (empty data)

    // Set display zone first
    SCR_set_col(ppg_pos, ppg_pos + NUM_OF_UPD_COLUMNS - 1);
    SCR_set_page(0, 0 + BYTES_IN_PPG_COLUMN - 1); // Always from top
    // Now buffer filled with:
    // [1] = SCR_MULTIBYTE_CMD etc.

    screen_buff[1] = SCR_MULTIBYTE_DATA;

    unsigned char col_byte = shift / SCR_1BYTE_DOTS; // The byte where point lies

    // Clear 2 columns
    for (unsigned char a = 2; a < (NUM_OF_UPD_COLUMNS * BYTES_IN_PPG_COLUMN + 2); ++a) {
        screen_buff[a] = 0;
    }

    // Draw 1st column filled, because:
    //  1) less code;
    //  2) it's not a cardiogram.
    for (unsigned char a = col_byte + 1 + 2; a < (BYTES_IN_PPG_COLUMN + 2); ++a) {
        screen_buff[a] = b_1111_1111; // Fill bytes before the point
    }

    screen_buff[col_byte + 2] = b_1111_1111 << (shift - SCR_1BYTE_DOTS * col_byte); // Fill point byte

    // Transfer data for 2 columns (+2 for TWI)
    TWI_Start_Transceiver_With_Data(&screen_buff[0], NUM_OF_UPD_COLUMNS * BYTES_IN_PPG_COLUMN + 2, TWI_MSG_IMP_LOW + TWI_MSG_STOP);
    TWI_Complete_Last_Transfer(SEND_STOP);
}

#ifdef MAX30102_EXT_01 // +352 bytes to code
static unsigned char request_tmpr_result = 0;

// Calculate and show temperature of the MAX sensor itself.
// Use case is limited because it always will be close to human body (finger) temperature.
// No calibration (as is).
void calc_sens_temperature(void) {
    SENS_get_die_tmpr_cfg();
    if (NOT_TMPR_EN) {
        // Previous temperature measurement finished.
        // Check if the result was requested.
        if (request_tmpr_result & b_0000_0001) {
            SENS_get_die_tmpr_value();

            // Smallest fraction = 0.0625 C
            unsigned short int frac_C = 625;
            frac_C *= TFRAC & SENS_TFRAC_MASK; // 625 * 0..15

            unsigned char last_drawpoint = 65;

            // T_measured = T_integer + T_fraction
            // Special case for negative numbers
            if (((char)TINT) < 0) { // T_integer < 0
                TINT = ~TINT; // Drop sign and subtract 1
                frac_C = 10000 - frac_C;
                // Draw "-"
                disp_str[0] = 0; // "-"
                disp_str[DIS_STR_NOD] = 1;
                last_drawpoint += SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, last_drawpoint + (3+1)*1-1, 1 | ALIGN_LEFT);
            }

            // Draw T_integer
            disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)TINT, 0);
            last_drawpoint += SCR_print_D(&disp_str[0], FONT_8PX, last_drawpoint, last_drawpoint + (3+1)*3-1, 1 | ALIGN_LEFT);

            disp_str[0] = 1; // "Decimal point"
            disp_str[DIS_STR_NOD] = 1;
            last_drawpoint += SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, last_drawpoint + (1+1)*1-1, 1 | ALIGN_LEFT);

            // Build T_fraction string
            frac_C += 500; // +0.0500, to round to 1 digit after decimal point (max 0.9375; no overflow)
            uitoa10(&disp_str[0], frac_C, 0);
            disp_str[DIS_STR_NOD] = 1; // Truncate to 1 digit

            // Force zero pudding for T_fraction
            if (TFRAC == 0) {
                disp_str[0] = 0;
                //disp_str[1] = 0;
                //disp_str[2] = 0;
                //disp_str[3] = 0;
                //disp_str[DIS_STR_NOD] = 4;
            } else if (TFRAC == 1) {
                disp_str[0] = 1; // Rounding
                //disp_str[0] = 0;
                //disp_str[1] = 6;
                //disp_str[2] = 2;
                //disp_str[3] = 5;
                //disp_str[DIS_STR_NOD] = 4;
            }

            // Draw T_fraction
            last_drawpoint += SCR_print_D(&disp_str[0], FONT_8PX, last_drawpoint, last_drawpoint + (3+1)*1-1, 1 | ALIGN_LEFT);

            disp_str[0] = 2; // "degree"
            disp_str[DIS_STR_NOD] = 1;
            last_drawpoint += SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, last_drawpoint + (2+1)*1-1, 1 | ALIGN_LEFT);

            disp_str[0] = 3; // "Celsius"
            disp_str[DIS_STR_NOD] = 1;
            //SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, last_drawpoint + (4+1)*1-1, 1 | ALIGN_LEFT);
            // In the end clear maximum possible space that shared between
            // drawings of temperature (30 px) and 1/R (30 px).
            SCR_print_D(&disp_str[0], SIGNS_8PX, last_drawpoint, 65 + (4+4*3+2+4+3+5)-1, 1 | ALIGN_LEFT);

            // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
            TWI_Complete_Last_Transfer(SEND_STOP);

            request_tmpr_result = 0;
        } else {
            // Run new temperature measurement and start asking for
            // result when it ready.
            SENS_set_die_tmpr_en();
            request_tmpr_result = 1;
        }
    } // NOT_TMPR_EN
}
#endif

// Switch pin to enable voltage divider and wait for voltage to stabilize
static void v_divider_on (void) {
    // Output low (clear PORT, set DDR)
    ADC_DIV_PORT &= ~ADC_DIV_CONNECT_MSK;
    ADC_DIV_DDR |= ADC_DIV_CONNECT_MSK;

    // Wait for voltage to stabilize, ~1 ms
    __builtin_avr_delay_cycles(1 * ONE_MSEC_CYCL);
}

// Switch pin to disable voltage divider
static void v_divider_off (void) {
    // Input tri-state (clear PORT, clear DDR)
    ADC_DIV_PORT &= ~ADC_DIV_CONNECT_MSK;
    ADC_DIV_DDR &= ~ADC_DIV_CONNECT_MSK;
}

// 10-bit
// ADC ->  V (divider = 4.7 + 1; 1.1V ref.)
// -----------
// 817 -> 5.0V
// 735 -> 4.5V
// 686 -> 4.2V
// 490 -> 3.0V
// 473 -> 2.9V
// 441 -> 2.7V
// V = Vdivider * (ADC * Vref) / 1024
//
// 8-bit
// ADCH ->  V (divider = 4.7 + 1; 1.1V ref.)
// -----------
// 204 -> 5.0V
// 183 -> 4.5V
// 171 -> 4.2V
// 122 -> 3.0V
// 118 -> 2.9V
// 110 -> 2.7V
// V = Vdivider * (ADC * Vref) / 256

#define ADC_8BIT_1V1REF_5DIV7 245  // 0.0244921875

// Display ADC as voltage
static void display_voltage (unsigned char adc_value) {
    unsigned short int v = ADC_8BIT_1V1REF_5DIV7;
    v *= adc_value;
    v += 500; // Rounding
    v /= 1000;

    unsigned char two_digits = v;
    unsigned char int_part = two_digits / 10;
    unsigned char decimal_part = two_digits % 10;

    // Display "d.dV" at top right corner
    disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)int_part, 0);
    SCR_print_D(&disp_str[0], FONT_8PX, 127+1-((3+1)*1+(1+1)+(3+1)*1+(5+1)), 127+1-((1+1)+(3+1)*1+(5+1))-1, 0 | ALIGN_RIGHT);
    disp_str[0] = 1; // "Decimal point"
    disp_str[DIS_STR_NOD] = 1;
    SCR_print_D(&disp_str[0], SIGNS_8PX, 127+1-((1+1)+(3+1)*1+(5+1)), 127+1-((3+1)*1+(5+1))-1, 0 | ALIGN_LEFT);
    disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)decimal_part, 0);
    SCR_print_D(&disp_str[0], FONT_8PX, 127+1-((3+1)*1+(5+1)), 127+1-((5+1))-1, 0 | ALIGN_LEFT);
    disp_str[0] = 5; // "Voltage"
    disp_str[DIS_STR_NOD] = 1;
    SCR_print_D(&disp_str[0], SIGNS_8PX, 127+1-((5+1)), 127+1-1, 0 | ALIGN_LEFT);

    // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
    TWI_Complete_Last_Transfer(SEND_STOP);
}

// ~125000 Hz ADC, 0.000928 sec for single measurement (25+13x7 ADC clocks)
// +0.001 sec to stabilize voltage divider. Total time = 0.001928 sec.
static void get_battery_voltage(void) {
    // Enable voltage divider
    v_divider_on();

    // Power up ADC
    PRR = (0 << PRADC) | (0 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM1)
          | (0 << PRTIM0) | (1 << PRTIM2) | (0 << PRTWI);

    // Disable digital input on ADC channel to reduce power consumption.
    // Other inputs are pulled-up, so no random switching expected.
    DIDR0 = (1 << ADC_CHANNEL); // (1 << ADC1D)

    // 1.1V ref. is ON, ADC1
    ADMUX = (1 << REFS1) | (1 << REFS0) | ADC_CHANNEL;

    unsigned short int result = 0;
    for (unsigned char a = 0; a < 8; ++a) {
        // Enable ADC, start conversion, division factor: 64
        ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1);
        while(!(ADCSRA & (1 << ADIF)));

        result += ADC; // Accumulate for averaging

        // Clear the ADC interrupt flag, keep ADC enabled, division factor: 64
        ADCSRA = (1 << ADEN) | (1 << ADIF) | (1 << ADPS2) | (1 << ADPS1);
    }

    // 10-bit to 8-bit -> ((result / 8) + 0.5 for rounding) / 4
    display_voltage((unsigned char)((result + 4) / 32));

    // 1.1V ref. is OFF, ADC1
    ADMUX = (1 << ADLAR) | ADC_CHANNEL;

    // Disable ADC for Power-down
    ADCSRA = (0 << ADEN);

    // Shut down (=1) unneeded peripherals
    PRR = (1 << PRADC) | (0 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM1)
          | (0 << PRTIM0) | (1 << PRTIM2) | (0 << PRTWI);

    // Disconnect voltage divider from GND
    v_divider_off();
}


int main(void) {
    // Set clock division factor = 1, running at 8 MHz, internal RC.
    // The power for PA chips can be lowered down to 2.7V.
    // Here the Assembler instructions used just to fit into 4 cycles limit.
    const unsigned char __pre_change1 = (1 << CLKPCE);
    const unsigned char __val = (0 << CLKPS3) |
                                (0 << CLKPS2) |
                                (0 << CLKPS1) |
                                (0 << CLKPS0);
    __asm__ __volatile__ (
        "sts %1, %0" "\n\t"
        "sts %1, %2"
        : // No outputs modified
        : "d" (__pre_change1),
          "M" (_SFR_MEM_ADDR(CLKPR)),
          "d" (__val)
        : // No tmp used
    );

    // Watchdog not in use
    wdt_disable();

PowerUP:;

    // Shut down (=1) unneeded peripherals
    PRR = (1 << PRADC) | (0 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM1)
          | (0 << PRTIM0) | (1 << PRTIM2) | (0 << PRTWI);

    // UART initial setup, should be performed on wake up too
    USART_Init(0); // 0 - Register value will be calculated automatically in uart_for_debug.c
                   //     The baud rate is defined in uart_for_debug.h
    stdout = &myout;

    // First debugging message. It will be sent from the MCU via UART.
    // printf() is heavy and has minimum implementation of ~1.4K that is
    // too much for a simple debugging purposes of the 8-bit MCU.
    // The fputs() will be used here + custom int_to_hex string conversion <400 bytes.
    fputs_P(PSTR("MCU started\n"), stdout);

    // Debouncing timer
    // Timer0, 1024 prescale, 0.000128 sec increment,
    // CTC mode, compareA is 0xEB (235 decimal) or 0.03008 sec;
    // compareB (that can be set < compareA) not in use;
    // 66 x debouncing = 2 sec hold.
#define DEB_TIMER_PRESCALE 1024
#define DEB_TIMER_TOP 0xEB

#if (DEB_TIMER_PRESCALE == 1024)
    TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
#elif (DEB_TIMER_PRESCALE == 256)
    TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00);
#elif (DEB_TIMER_PRESCALE == 64)
    TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
#elif (DEB_TIMER_PRESCALE == 8)
    TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);
#elif (DEB_TIMER_PRESCALE == 1)
    TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
#endif

// Frequency of Timer's clock, Hz
#define FREQ_TIMER_Hz (F_CPU / DEB_TIMER_PRESCALE)
#define FREQ_TIMER_TOP_Hz (FREQ_TIMER_Hz / DEB_TIMER_TOP)

// Button's max hold time, in timer's top_match events
#define HOLD_THRESHOLD (FREQ_TIMER_TOP_Hz * HOLD_THRESHOLD_MSEC / 1000)
#define HOLD_PWR_THRESHOLD (FREQ_TIMER_TOP_Hz * HOLD_PWR_THRESHOLD_MSEC / 1000)

#if (HOLD_THRESHOLD < 2)
 #error "HOLD_THRESHOLD_MSEC is too low for the selected prescale and top."
#elif (HOLD_THRESHOLD > 255)
 #error "HOLD_THRESHOLD_MSEC is too high for the selected prescale and top."
#endif

#if (HOLD_PWR_THRESHOLD > 255)
 #error "HOLD_PWR_THRESHOLD_MSEC is too high for the selected prescale and top."
#endif
    OCR0A = DEB_TIMER_TOP;
    //OCR0B = 0x75; // 0.014976 sec, not in use
    TCCR0A |= (1 << WGM01);
    TIMSK0 |= (1 << OCIE0A);
    TCNT0 = 0x00;
    PRR |= (1 << PRTIM0); // Shutdown the timer

    // All unused pins are pulled up internally (set Port, DDR = 0 by default).
    // All buttons pins are pulled up internally too.
    // Keep TWI, ADC channel and voltage divider connect pins in tri-state (DDR = 0, Port = 0 by default).
    //! Selected TWI, ADC and divider are on port C
    //! Power up ext HW on port D can remain pulled-up (external devices off)
    PORTB = b_1111_1111;
    PORTC = b_1111_1111 & (~((1 << SDA_BIT) | (1 << SCL_BIT) | ADC_DIV_CONNECT_MSK | ADC_CHANNEL_PORT_MSK));
    PORTD = b_1111_1111;

    // Enable the buttons pins for interrupts
    BTN_PCMSK |= ALL_BTN_MSK;
    PCICR |= (1 << BTN_PCIE);

    fputs_P(PSTR("MCU config ended\n"), stdout);

    // Next definitions used as single-press key lockers for buttons events
#define BTNS_RELEASED 0
#define BT1_PRESSED   1
#define BT1_HOLD      2
#define BT2_PRESSED   3
#define BT2_HOLD      4
#define BTNS_PRESSED_BT2_HOLD 252
#define BTNS_PRESSED_BT1_HOLD 253
#define BTNS_PRESSED  254
#define BTNS_HOLD     255
    unsigned char btn_old = BTNS_RELEASED;

    // Power up I2C devices and wait for voltage to stabilize
    power_up_ext_hw();

    // TWI initial setup, 400kHz, should be performed on wake up too
    TWI_Master_Initialize();

    sei();

    scan_twi(); // Look for all available devices

    SCR_wakeup();

    // Visual screen dots check on MCU reset
    SCR_set_all_dots_on(1);
    __builtin_avr_delay_cycles(500 * ONE_MSEC_CYCL); // ~500 ms (includes I2C transfer time)
    SCR_set_all_dots_on(0);

DrawInitInfo:;

    // Show battery statistic
    get_battery_voltage();

    // Show current range/scale in "hearts" (no/empty/half/full)
    if (fs_val) {
        disp_str[0] = 0; // "heart"
        disp_str[DIS_STR_NOD] = 1;
        SCR_print_D(&disp_str[0], IMGS_8PX, 65, 65+9-1, 0 | ALIGN_LEFT);
        const unsigned char sc_width = fs_val >> SENS_SPO2_ADC_RGE_BITS;
        //  >>  01 10 11
        //  -1   0  1  2
        //  -------------
        // glyph 0  1  2
        disp_str[0] = sc_width - 1; // Empty(0)/Half(1)/Full(2) filled "heart"
        //  >>  01 10 11
        //  &1  01 00 01
        //  x4   4  0  4
        //  +5   9  5  9
        // -------------
        // width 9  5  9
        disp_str[DIS_STR_NOD] = 1;
        SCR_print_D(&disp_str[0], IMGS_8PX, 65, 65+5+((sc_width & b_0000_0001) * 4)-1, 0 | ALIGN_LEFT);
    } else {
        SCR_clear_rect(65, 65+9-1, 0, 0); // Display nothing (no "heart")
    }

    // Show LED "Brightness"
    disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)LED1_current, 0);
    SCR_print_D(&disp_str[0], FONT_8PX, 65+9, 65+9+(3+1)*3-1, 0 | ALIGN_LEFT);

    // Display big heart
    disp_str[0] = sens_mode & b_0000_0001; // "heart" for HR, "solid_heart" for SpO2
    disp_str[DIS_STR_NOD] = 1;
    SCR_print_D(&disp_str[0], IMGS_24PX, 100, 100+23-1, 1 | ALIGN_LEFT);

    // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
    TWI_Complete_Last_Transfer(SEND_STOP);

    // Get sensor revision and ID
    if (!SENS_get_rev_id()) {
        fputs_P(PSTR("!Ei\n"), stdout); // Error getting info
    }

    if (PART_ID != MAX30102_PART_ID) {
        fputs_P(PSTR("Unknown "), stdout);
    }

    fputs_P(PSTR("MAX sensor:\n"), stdout);
    fputs_P(PSTR(" rev=0x"), stdout);
    fputs(usitohex(&disp_str[0], (unsigned short int)REV_ID), stdout);
    fputs_P(LF, stdout);
    fputs_P(PSTR(" id=0x"), stdout);
    fputs(usitohex(&disp_str[0], (unsigned short int)PART_ID), stdout);
    fputs_P(LF, stdout);

    sens_reset_to_HR_SpO2(sens_mode);

    unsigned char ready_smpls = 0; // Number of ready to read samples (inside the sensor)
    unsigned char n; // Number of bytes to read from the sensor

// Number of samples that fits into the TWI buffer
#define BUFF_TWI_SMPLS_NUM_HR ((TWI_BUFFER_SIZE - 1) / SENS_SMPL_LNGTH_HR)
#define BUFF_TWI_SMPLS_NUM_SpO2 ((TWI_BUFFER_SIZE - 1) / SENS_SMPL_LNGTH_SpO2)

// Lowest number (for largest samples)
#if (BUFF_TWI_MAX_SMPLS_NUM_SpO2 < BUFF_TWI_SMPLS_NUM_HR)
 #define BUFF_TWI_SMPLS BUFF_TWI_SMPLS_NUM_SpO2
#else
 #define BUFF_TWI_SMPLS BUFF_TWI_SMPLS_NUM_HR
#endif
#if (BUFF_TWI_SMPLS < 1)
    #error "TWI_BUFFER_SIZE is too small to hold at least 1 sample"
#endif

//! Minimum/maximum signal strength for robust detection
#define AC_STRENGTH 100
#define EXTREMUM_STRENGTH (AC_STRENGTH * 2 / 3) // 66% of AC_STRENGTH
#define SMPL_TOP_TRESHOLD (UINT16_MAX - UINT16_MAX / 16) // ~94%
#define SMPL_BOTTOM_TRESHOLD (UINT16_MAX / 3)            // ~33%

// Coefficients for exponential smoothing
//
// Delta_t = 1 / SR
// T = 1 / low_pass_filter_edge_frequency
// Alpha = 1 - Exp(-Delta_t / T)
// For SR = 50 smpl/sec, max freq is twice lower = 50 / 2 = 25 beats/sec (1500 beats/min)
// For 16.66 smpl/sec display rate, max freq = 16.66 / 2 = 8.33 beats/sec (500 beats/min)
// Closest upper (integer) frequency for 8.33 is 9 Hz.
#define RECIPROCAL_ALPHA_6Hz  (SR_VALUE_SAMPLES / 6)  // [6], >> 3
#define RECIPROCAL_ALPHA_7Hz  (SR_VALUE_SAMPLES / 7)  // [7]
#define RECIPROCAL_ALPHA_8Hz  (SR_VALUE_SAMPLES / 8)  // [8]
#define RECIPROCAL_ALPHA_9Hz  (SR_VALUE_SAMPLES / 10) // [9..10], ~9.11607784 Hz for 50 smpl/sec
#define RECIPROCAL_ALPHA_11Hz (SR_VALUE_SAMPLES / 12) // [11..12], >> 2
#define RECIPROCAL_ALPHA_14Hz (SR_VALUE_SAMPLES / 16) // [13..16]
#define RECIPROCAL_ALPHA_20Hz (SR_VALUE_SAMPLES / 25) // [17..25], >> 1

    short int smpls_diff[PEAK_WND_WIDTH] = {0}; // Differences between two sequential samples for LED1
    unsigned short int prev_smpl_LED1 = 0; // Previous value for LED1
    unsigned short int prev_smpl_LED2 = 0; // Previous value for LED2
    unsigned short int extremum[4] = {0}; // [0] - LED1 max , [1] - LED1 min,
                                          // [2] - LED2 max , [3] - LED2 min.
#define LED1_MAX extremum[0]
#define LED1_MIN extremum[1] // Systolic (blood rich)
#define LED2_MAX extremum[2]
#define LED2_MIN extremum[3] // Systolic (blood rich)

    //             max1
    //              +           +           +
    //           +   +       +   +       +   +
    //        +       +   +       +   +       +
    //     +           +           +           +
    //    min1(old)   min1

    // Cycled storage to keep the N-th sample of the LED2
    unsigned short int smpl_LED2[PEAK_WND_WIDTH] = {0};

    // In window index (cycled within PEAK_WND_WIDTH borders)
    // shared for both samples and sum differences.
    unsigned char inwnd_idx = 0;

    // Sum of the differences in the window for LED1 (it needed to find presence of the extremum in the window).
    // LED2 not analyzed becase both LEDs are in sync.
    short int sum_diff = 0;
    short int prev_sum_diff = 0;

    // Next two counters are same for both LEDs, so using only one pair without specifying the index
    unsigned short int min_old_to_max_dist = 0; // distance from min_old to max
    unsigned short int current_dist = 0; // current distance counter,
                                         // counts distance (in samples) from previous extremum to next

    // AC/DC characteristics
    unsigned short int acdc[4] = {AC_STRENGTH}; // [0] - AC LED1, [1] - DC LED1,
                                                // [2] - AC LED2, [3] - DC LED2.
#define AC_LED1 acdc[0]
#define DC_LED1 acdc[1]
#define AC_LED2 acdc[2]
#define DC_LED2 acdc[3]

    unsigned char find_min = 0; // All initial values in window are 0, so look for maximum first

// At least 3 sec of PPG should be shown. PPG width is half of the screen width.
// 64 px / 3 sec = 21.33 px/s (1 px = 1 smpl).
// Closest common divider for all sample rates is 16.66.
// It can show 64 px / 16.66 px/s = 3.84 sec of PPG (1 px = 1 smpl).
//   15 sec = 250 px shown at 16.66 px/sec ( 50 smpl/sec, divider 3);
//   15 sec = 250 px shown at 16.66 px/sec (100 smpl/sec, divider 6);
//   10 sec = 200 px shown at 20 px/sec    ( 50 smpl/sec, divider 2.5); - not integer divider
//   10 sec = 200 px shown at 20 px/sec    (100 smpl/sec, divider 5).
#define RATE_OF_PPG_DISPLAY (SR_VALUE_SAMPLES * 3 / 50) // N-th sample that ensures 16.66 px/sec display rate
#define BEATS_COUNT_SEC 15
#define COUNTS_IN_1MINUTE (60 / BEATS_COUNT_SEC) // 4 times
#define AVERAGING_TIMEOUT_px (BEATS_COUNT_SEC * 50 / 3) // 15 sec * 16.66 px/sec = 250 px

    unsigned char beats_counter = 0; // Actual heart rate for the given time interval
    unsigned char quarter_beats[COUNTS_IN_1MINUTE] = {0}; // Beats counters for each 15 sec intervals
    unsigned char sum_beats = 0; // Partial sum of quarter_beats

    unsigned char smpls_counter = 0; // To detect each N-th sample only
    unsigned char smpls_diplayed = 0; // To count 15 sec intervals

    unsigned char qb_count = 0; // Number of continuous 15 sec intervals
    unsigned char qb_idx = 0; // Indexes of the cycled buffer

    unsigned char show_oxy = 0; // Flag for SpO2 calculation and display
    unsigned char ppg_pos = 0; // Live position of the PPG line

    // Main loop
    while(1) {
        // Pin interrupt will power up the debouncing timer,
        // so make shutdown of the timer "atomic".
        // TWI here is interrupt driven, so it is undesirable to turn off
        // the global interrupts (to not miss the TWI bus event or break
        // active transfer).
        //
        // Exclude button pins from the interrupt scan.
        BTN_PCMSK &= ~ALL_BTN_MSK;
        if (debounce_complete &&    // At least 1 debouncing ended
            (bt1 | bt2) == 0) {     // All buttons released

            // Reset and shutdown the debouncing timer
            TCNT0 = 0x00;
            PRR |= (1 << PRTIM0);
            debounce_complete = 0;
        }

        // Add button pins to the interrupt scan
        BTN_PCMSK |= ALL_BTN_MSK;

        // --- Get samples from the sensor ---

        // Get FIFO pointers
        if (SENS_get_FIFO_pointers()) {
            ready_smpls = FIFO_WR_PTR;
            ready_smpls -= FIFO_RD_PTR;
            ready_smpls &= b_0001_1111; // 5 bit long
            // Workaround possible stalls on bus errors (FIFO full)
            if (OVF_COUNTER > 0) {
                ready_smpls = BUFF_TWI_SMPLS;
            }
        } else {
            fputs_P(PSTR("!Ep\n"), stdout); // Error getting pointers
            ready_smpls = 0;
        }

        // Show number of samples ready to read
        disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)ready_smpls, 0);
        SCR_print_D(&disp_str[0], FONT_8PX, 65, 65+(3+1)*2-1, 3 | ALIGN_LEFT);

        // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
        TWI_Complete_Last_Transfer(SEND_STOP);

        // Limit number of samples to read
        if (ready_smpls > BUFF_TWI_SMPLS) {
            ready_smpls = BUFF_TWI_SMPLS;
        }

        n = ready_smpls * SENS_SMPL_LNGTH_SpO2; // Number of bytes to read

        if (sens_mode == SENS_MODE_HEART_RATE) {
            n = ready_smpls * SENS_SMPL_LNGTH_HR; // Number of bytes to read
        }

        // Skip data processing if nothing to read
        if (n == 0) {
            goto ProcessButtons;
        }

        // Get n bytes of data from the FIFO
        N_BYTES = n;
        if (!SENS_get_FIFO_N_bytes()) {
            fputs_P(PSTR("!Ed\n"), stdout); // Error getting data
            n = 0;
        }

        // Total number of 3-byte data to process (can be twice higher than the ready_smpls in SpO2 mode)
        unsigned char total = n / S_SIZE;
        unsigned short int* s16_p; // Pointer to the 16-bit sample

        for (unsigned char i = 1; i <= total; ++i) {
            // Convert sample data to number
            bytedata24be_to_16le(i);

            // Print sample to console
            s16_p = (unsigned short int*)&sens_buff[i * S_SIZE - 2];
            fputs(usitohex(&disp_str[0], *s16_p), stdout);
            fputs_P(LF, stdout);
        }

        // --- Process all samples ---

        for (unsigned char i = 1; i <= total; ++i) {
            // Advance the interval counter
            ++current_dist;

            // LED1 sample pointer
            s16_p = (unsigned short int*)&sens_buff[i * S_SIZE - 2];

            // Advance to LED2 sample in SpO2 mode, convert data
            if (sens_mode == SENS_MODE_SPO2) {
                ++i;
            }

            // Remember sample value. In case of SpO2 mode -> it stores LED2 value,
            // otherwise stores value that is same to the LED1 (*s16_p).
            smpl_LED2[inwnd_idx] = *((unsigned short int*)&sens_buff[i * S_SIZE - 2]);

            // Exponential filter
            short int tmp_diff = (short int)(smpl_LED2[inwnd_idx] - prev_smpl_LED2);
            if (tmp_diff < 0) {
                // Use less strict filter on back of the signal
                smpl_LED2[inwnd_idx] = (unsigned short int)(prev_smpl_LED2 + tmp_diff / RECIPROCAL_ALPHA_20Hz); //! unsigned + signed
            } else {
                smpl_LED2[inwnd_idx] = (unsigned short int)(prev_smpl_LED2 + tmp_diff / RECIPROCAL_ALPHA_9Hz); //! unsigned + signed
            }
            prev_smpl_LED2 = smpl_LED2[inwnd_idx];

            // Display PPG
            ++smpls_counter;
            if (smpls_counter > (RATE_OF_PPG_DISPLAY - 1)) { // Each 3rd (~16.66 per second, ~3.84 sec on display)
                render_ppg(smpl_LED2[inwnd_idx], AC_LED2, DC_LED2, ppg_pos);
                ++smpls_diplayed;
                ++ppg_pos;
                if (ppg_pos > PPG_XMAX) {
                    ppg_pos = 0; // Wrap it, but (PPG_XMAX + 1)th column will be cleared
                }
                smpls_counter = 0;
            }

            // Average heart rate (each 15 sec) and display it
            if (smpls_diplayed == AVERAGING_TIMEOUT_px) {
                sum_beats -= quarter_beats[qb_idx];
                quarter_beats[qb_idx] = beats_counter; // Save beats each 15 sec
                sum_beats += beats_counter;

                // Divider with saturation.
                // 4 iterations --> 1 minute.
                if (qb_count < COUNTS_IN_1MINUTE) {
                    ++qb_count;
                }

                // "60 / BEATS_COUNT_SEC / qb_count" turns into 1 after COUNTS_IN_1MINUTE iterations
                unsigned short int timed_heart_rate = (unsigned short int)sum_beats * (60 / BEATS_COUNT_SEC) / qb_count;

                disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], timed_heart_rate, 0);
                SCR_print_D(&disp_str[0], FONT_32PX, 127+1-(18+3)*3, 127, 4 | ALIGN_RIGHT);
                TWI_Complete_Last_Transfer(SEND_STOP);

#ifdef MAX30102_EXT_01
                // Show crystal temperature each 30 sec (1st call - request, 2nd - value)
                calc_sens_temperature();
#endif

                // Flag SpO2 for calculation and display
                show_oxy = 1;

                ++qb_idx;
                if (qb_idx > 3) {
                    qb_idx = 0; // Wrap indexes
                    get_battery_voltage(); // Once in a minute
                }

                beats_counter = 0;
                smpls_diplayed = 0;

            } // AVERAGING_TIMEOUT_px

            // Skip low level samples, data unreliable
            if (smpl_LED2[inwnd_idx] < SMPL_BOTTOM_TRESHOLD) {
                beats_counter = 0;
                qb_count = 0;
                qb_idx = 0;

                // Clear beats counter
                sum_beats = 0;
                smpls_diplayed = 0;
                for (unsigned char a = 0; a < 4; ++a) {
                    quarter_beats[a] = 0;
                }

                // Clear instant heart rate from display
                SCR_clear_rect(85, 85+(3+1)*3-1, 2, 2);

                // Clear SpO2 and timed heart rate from display.
                // TWI transfer here takes ~12 ms, so sensor's buffer
                // likely overflow on few consecutive low level samples.
                SCR_clear_rect(0, 127, 4, 7);
                continue;
            }

            prev_sum_diff = sum_diff;

            // Subtract oldest value
            sum_diff -= smpls_diff[inwnd_idx];

            unsigned short int current_smpl_LED1; // Not filtered sample
            current_smpl_LED1 = *s16_p;

            // Exponential filter
            smpls_diff[inwnd_idx] = (short int)(current_smpl_LED1 - prev_smpl_LED1);
            if (smpls_diff[inwnd_idx] < 0) {
                // Use less strict filter on back of the signal.
                current_smpl_LED1 = (unsigned short int)(prev_smpl_LED1 + smpls_diff[inwnd_idx] / RECIPROCAL_ALPHA_20Hz); //! unsigned + signed
            } else {
                current_smpl_LED1 = (unsigned short int)(prev_smpl_LED1 + smpls_diff[inwnd_idx] / RECIPROCAL_ALPHA_9Hz); //! unsigned + signed
            }

            // Smoothed samples difference
            smpls_diff[inwnd_idx] = (short int)(current_smpl_LED1 - prev_smpl_LED1);

            // Add recent value
            sum_diff += smpls_diff[inwnd_idx];

            prev_smpl_LED1 = current_smpl_LED1;

            // Look for extremums
            if (find_min) {
                if (prev_sum_diff < 0 && sum_diff >= 0) {
                    // Rise (moving from - to +)
                    unsigned char extremum_pos; // Position of the extremum in the window

                    // Find position of the minimum in the window for LED2
                    extremum_pos = find_extremum_pos(find_min, inwnd_idx, current_dist, smpl_LED2);

                    unsigned char n_smpls_ago; // In window distance, N-samples ago
                    unsigned short int extr_tmp;

                    // Restore minimum's value from current value and all preceding differences
                    extr_tmp = current_smpl_LED1;
                    n_smpls_ago = restore_smpl_value(&extr_tmp, smpls_diff, extremum_pos, inwnd_idx);

                    // Skip weak signals and ensure that current min lower enough than last max.
                    // In case positions of LED1<>LED2 extremums not match -> signal is noised.
                    if (LED1_MAX > (extr_tmp + EXTREMUM_STRENGTH)) {

                        // --- LED1 ---

                        DC_LED1 = LED1_MIN; // Remember old minimum
                        LED1_MIN = extr_tmp; // Update with new minimum

                        // Actual distance
                        current_dist -= n_smpls_ago;

                        //! Prevent division by zero in DC calculations
                        if ((min_old_to_max_dist + current_dist) == 0) {
                            current_dist = 1;
                            min_old_to_max_dist = 9;
                        }

                        // Be aware that the (old_min - min) * interval
                        // should fit into the type "unsigned short int".
                        //
                        // Here DC_LED1 stores old minimum that will be updated
                        // with actual DC value of the LED1.
                        calc_dc(&DC_LED1, LED1_MIN, min_old_to_max_dist, current_dist);

                        // AC
                        AC_LED1 = LED1_MAX - DC_LED1;

                        // --- LED2 ---

                        DC_LED2 = LED2_MIN; // Remember old minimum
                        LED2_MIN = smpl_LED2[extremum_pos]; // Update minimum

                        // Here DC_LED2 stores LED2 old minimum that will be updated
                        // with actual DC value of the LED2.
                        calc_dc(&DC_LED2, LED2_MIN, min_old_to_max_dist, current_dist);

                        // AC
                        AC_LED2 = LED2_MAX - DC_LED2;

                        // Display instant heart rate
                        unsigned char heart_rate = calc_heart_rate(min_old_to_max_dist + current_dist);
                        disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)heart_rate, 0);
                        SCR_print_D(&disp_str[0], FONT_8PX, 85, 85+(3+1)*3-1, 2 | ALIGN_RIGHT);

                        // Complete SCR transfer and send STOP or ADC in MAX IC wouldn't start
                        TWI_Complete_Last_Transfer(SEND_STOP);

                        if (show_oxy && (sens_mode == SENS_MODE_SPO2)) {
                            calc_r(AC_LED1, DC_LED1, AC_LED2, DC_LED2);
                            show_oxy = 0; // One in 15 sec
                        }

                        ++beats_counter;

                        current_dist = n_smpls_ago; //!!! Restore counter
                    } // EXTREMUM_STRENGTH

                    find_min = 0; // Next is maximum
                }
            } else { // Looking for max
                if (prev_sum_diff >= 0 && sum_diff < 0) {
                    // Fall (moving from + to -)
                    unsigned char extremum_pos; // Position of the extremum in the window

                    // Find position of the maximum in the window for LED2
                    extremum_pos = find_extremum_pos(find_min, inwnd_idx, current_dist, smpl_LED2);

                    unsigned char n_smpls_ago; // In window distance, N-samples ago
                    unsigned short int extr_tmp;

                    // Restore maximum's value from current value and all preceding differences
                    extr_tmp = current_smpl_LED1;
                    n_smpls_ago = restore_smpl_value(&extr_tmp, smpls_diff, extremum_pos, inwnd_idx);

                    // Skip weak signals and ensure that current max much bigger than last min.
                    // In case positions of LED1<>LED2 extremums not match -> signal is noised.
                    if (extr_tmp > (LED1_MIN + EXTREMUM_STRENGTH)) {
                        LED1_MAX = extr_tmp; // Remember only strong extremums

                        // Actual distance
                        current_dist -= n_smpls_ago;

                        LED2_MAX = smpl_LED2[extremum_pos];

                        min_old_to_max_dist = current_dist;
                        current_dist = n_smpls_ago; //!!! Restore counter
                    } // EXTREMUM_STRENGTH
                    find_min = 1; // Next is minimum
                }
            } // Find max

            // Cycle through the elements of the window.
            // Next element is oldest.
            ++inwnd_idx;
            if (inwnd_idx >= PEAK_WND_WIDTH) {
                inwnd_idx = 0;
            }
        } // Process all samples

ProcessButtons:;

        // Buttons
        if ((bt1 | bt2) > 0) { // Either one is pressed
            if (bt1 > 0 && bt2 > 0) {
                btn_old = BTNS_PRESSED;
                // Both buttons pressed...
                if (bt1 >= HOLD_THRESHOLD && bt2 < HOLD_THRESHOLD) {
                    btn_old = BTNS_PRESSED_BT1_HOLD;
                    // ...and Button 1 hold
                } else if (bt2 >= HOLD_THRESHOLD && bt1 < HOLD_THRESHOLD) {
                    btn_old = BTNS_PRESSED_BT2_HOLD;
                    // ...and Button 2 hold
                } else if (bt1 >= HOLD_THRESHOLD && bt2 >= HOLD_THRESHOLD) {
                    btn_old = BTNS_HOLD;
                    // ...and both buttons hold
                }
            } else { // Only one is pressed
                if (bt1 > 0 && btn_old == BTNS_RELEASED) {
                    btn_old = BT1_PRESSED;
                    // Button 1 pressed
                } else if (bt1 >= HOLD_THRESHOLD && btn_old == BT1_PRESSED) {
                    btn_old = BT1_HOLD;
                    // Button 1 hold
                    // Switch to HR mode, redraw UI
                    sens_mode = SENS_MODE_HEART_RATE;
                    goto DrawInitInfo;
                } else if (bt2 > 0 && btn_old == BTNS_RELEASED) {
                    btn_old = BT2_PRESSED;
                    // Button 2 pressed
                } else if (bt2 >= HOLD_THRESHOLD && btn_old == BT2_PRESSED) {
                    btn_old = BT2_HOLD;
                    // Button 2 hold
                    // When already in SpO2 mode adjust sensor scale
                    if (sens_mode == SENS_MODE_SPO2) {
                        // Make sensor less sensitive (increase full scale)
                        fs_val += SENS_SPO2_ADC_RGE_4096;
                        fs_val &= SENS_SPO2_ADC_RGE_MASK;

                        // Adjust current
                        LED1_current = curr_per_range[fs_val >> SENS_SPO2_ADC_RGE_BITS];
                        LED2_current = LED1_current;
                    }

                    // Switch to SpO2 mode, redraw UI
                    sens_mode = SENS_MODE_SPO2;
                    goto DrawInitInfo;
                }
            }
        } else { // All buttons were already released
            if (btn_old == BT1_PRESSED) {
                // Button 1 was pressed and now released
                // Decrease LED1, LED2 current
                if (LED1_current != 0) {
                    --LED1_current;
                }

                if (LED2_current != 0) {
                    --LED2_current;
                }

                LED1_PA = LED1_current;
                LED2_PA = LED2_current;
                SENS_set_LED_current();

                // Show updated LED "Brightness"
                disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)LED1_current, 0);
                SCR_print_D(&disp_str[0], FONT_8PX, 65+9, 65+9+(3+1)*3-1, 0 | ALIGN_LEFT);
                TWI_Complete_Last_Transfer(SEND_STOP);
            } else if (btn_old == BT2_PRESSED) {
                // Button 2 was pressed and now released
                // Increase LED1, LED2 current
                if (LED1_current != 0xFF) {
                    ++LED1_current;
                }

                if (LED2_current != 0xFF) {
                   ++LED2_current;
                }

                LED1_PA = LED1_current;
                LED2_PA = LED2_current;
                SENS_set_LED_current();

                // Show updated LED "Brightness"
                disp_str[DIS_STR_NOD] = uitoa10(&disp_str[0], (unsigned short int)LED1_current, 0);
                SCR_print_D(&disp_str[0], FONT_8PX, 65+9, 65+9+(3+1)*3-1, 0 | ALIGN_LEFT);
                TWI_Complete_Last_Transfer(SEND_STOP);
            } else if (btn_old == BTNS_PRESSED) {
                // Both Buttons were pressed and now both released
            }
            btn_old = BTNS_RELEASED;
        }

        // On/off procedures
        if (bt1 >= HOLD_PWR_THRESHOLD && btn_old == BT1_HOLD) {
            // Button 1 was hold 3 sec
            // Prepare to sleep MCU
            fputs_P(PSTR("sleep\n"), stdout);
            power_down_ext_hw();

            // Wait for button release, ~2 sec (to not wake up immediately after entering sleep mode)
            __builtin_avr_delay_cycles(2000 * ONE_MSEC_CYCL);

            do {
                cli(); // Ensure that nothing will interrupt MCU on the way to sleep
                set_sleep_mode(SLEEP_MODE_PWR_DOWN);

                // Shut down (=1) unneeded peripherals
                PRR = (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM1)
                      | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRTWI);

                bt1 = 0;
                sleep_enable();
                sleep_bod_disable(); // BOD not needed, on wake up almost full restart expected
                sei();
                sleep_cpu(); // Now sleeping
                sleep_disable(); // Wake up here (interrupt should power up timer to process debouncing)

                // Wait at least for 1 debouncing period to get true value of bt1 in the next step
                __builtin_avr_delay_cycles(1000 / FREQ_TIMER_TOP_Hz * ONE_MSEC_CYCL + ONE_MSEC_CYCL);

                // Pause to ensure that button was hold long enough to power up device
                while(bt1 && (bt1 < HOLD_PWR_THRESHOLD)) {
                    // Do nothing
                }
            } while (bt1 < HOLD_PWR_THRESHOLD); // Return to sleep if button was released prematurely

            bt1 = 0;
            bt2 = 0;
            goto PowerUP;
        } // On/off

#define WAIT_SMPLS 2 // Max = 21 for 130 (129 effective) bytes buffer
#if (WAIT_FOR_N_SMPLS > BUFF_TWI_SMPLS)
 #error "Wait time too long - buffer cannot store so many samples"
#endif
    // Wait for new samples to arrive, 2 samples or ~40 ms
    __builtin_avr_delay_cycles(WAIT_SMPLS * 1000 / SR_VALUE_SAMPLES * ONE_MSEC_CYCL);
    }; // while(1)

    return 0;
}

// --- Interrupts handling ---

#if (BUTTON_1_PORT == 0xB)
ISR (PCINT0_vect) {
#elif (BUTTON_1_PORT == 0xC)
ISR (PCINT1_vect) {
#elif (BUTTON_1_PORT == 0xD)
ISR (PCINT2_vect) {
#endif
    // The button logical state will be updated after the
    // debouncing timeout outside of this interrupt.
    // Button press event that is shorter than the timer's timeout
    // will be missed.

    // Power up and reset the debouncing timer.
    // NOTE: Interrupts during debouncing timeout extends the timeout itself.
    PRR &= ~(1 << PRTIM0);
    TCNT0 = 0x00;
    TIMSK0 |= (1 << OCIE0A);
    debounce_complete = 0;
}

ISR (TIMER0_COMPA_vect) {
    // Debouncing timeout

    // Remember all pins states at one point.
    // This synchronizes each button state to one MCU cycle,
    // no matter how long MCU will analyze the buttons states.
    // Re-use variable, so temporary debounce_complete = pins_state.
    debounce_complete = BTN_PORT_PINS;
    bt1 = get_button_state(BUTTON_1_MSK, 1);
    bt2 = get_button_state(BUTTON_2_MSK, 2);

    debounce_complete = 1;
}
