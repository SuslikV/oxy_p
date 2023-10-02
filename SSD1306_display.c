/*
 * Control the OLED dispaly based on SSD1306 driver.
 * Monochrome 128x64 or 128x32 dot matrix display.
 * Communications via I2C interface (TWI_Master.c file, M20230827).
 *
 * Version: M20230827
 */

#include "TWI_Master.h"      // I2C master driver
#include "SSD1306_display.h" // Display device specific definitions
#include "fonts_symbols.h"   // Glyphs and images storage

// Screen buffer. Not less than 5 bytes. Up to 255 bytes. By default set to GDDRAM page size +
// + 1 for address + 1 for command/data = 130 bytes.
extern unsigned char screen_buff[];


/****************************** Commands ******************************/

                        /********************/
                        /** 1. Fundamental **/
                        /********************/
////////////////////////////////////////////////////////////////////////
// Call this function to set display contrast
// ---------------------------------------------------------------------
// contrast = 0..255, max light of the each dot [RESET = 31]
////////////////////////////////////////////////////////////////////////

void SCR_set_contrast(unsigned char contrast) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_SET_CONTRAST;
    screen_buff[3] = contrast;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


#ifdef SSD1306_EXT_01
////////////////////////////////////////////////////////////////////////
// Call this function to set display GDDRAM content
// ---------------------------------------------------------------------
// all_on = 0, light only dots that are set in the GDDRAM [RESET]
// all_on = 1, light on all dots of the display
////////////////////////////////////////////////////////////////////////

void SCR_set_all_dots_on(unsigned char all_on) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_FROM_RAM_CMD | (all_on & b_0000_0001);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set normal/inverse display
// ---------------------------------------------------------------------
// inverse = 0, each 1 in GDDRAM is lighten up [RESET]
// inverse = 1, each 0 in GDDRAM is lighten up
////////////////////////////////////////////////////////////////////////

void SCR_set_inverse(unsigned char inverse) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_INVERSE_CMD | (inverse & b_0000_0001);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


////////////////////////////////////////////////////////////////////////
// Call this function to set display ON/OFF
// ---------------------------------------------------------------------
// power_on = 0, sleep mode (panel OFF) [RESET]
// power_on = 1, normal operation
////////////////////////////////////////////////////////////////////////

void SCR_set_panel_power_on(unsigned char power_on) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_POWER_CMD | (power_on & b_0000_0001);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


                         /******************/
                         /** 2. Scrolling **/
                         /******************/
// TODO: add commands


                    /***************************/
                    /** 3. Addressing setting **/
                    /***************************/
#ifdef SSD1306_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM column start address (L), page mode
// ---------------------------------------------------------------------
// Only for page addressing mode
//
// col_start_L = 0..15, start column (lower nibble)
// [RESET = 0]
////////////////////////////////////////////////////////////////////////

void SCR_set_col_L(unsigned char col_start_L) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_LOW_N_COL_CMD | (col_start_L & b_0000_1111);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM column start address (H), page mode.
// Suitable to move with x16 increments inside single page.
// ---------------------------------------------------------------------
// Only for page addressing mode
//
// col_start_H = 0..15, start column (higher nibble)
// [RESET = 0]
////////////////////////////////////////////////////////////////////////

void SCR_set_col_H(unsigned char col_start_H) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_HIGH_N_COL_CMD | (col_start_H & b_0000_1111);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM accessing mode
// ---------------------------------------------------------------------
// mode = 0 (SCR_MEM_HORIZ_MODE), horizontal addressing
// mode = 1 (SCR_MEM_VERT_MODE), vertical addressing
// mode = 2 (SCR_MEM_PAGE_MODE), page addressing [RESET]
////////////////////////////////////////////////////////////////////////

void SCR_set_mem_mode(unsigned char mode) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_MEM_MODE;
    screen_buff[3] = mode;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM column start/end address
// ---------------------------------------------------------------------
// Only for horizontal/vertical addressing mode
//
// col_start  = 0..127, start column
// col_end    = 0..127, end column
// [RESET: col_start = 0, col_end = 127]
////////////////////////////////////////////////////////////////////////

void SCR_set_col(unsigned char col_start, unsigned char col_end) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_COLUMNS;
    screen_buff[3] = col_start;
    screen_buff[4] = col_end;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 5, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM page start/end address
// ---------------------------------------------------------------------
// Only for horizontal/vertical addressing mode
//
// page_start = 0..7, start page
// page_end   = 0..7, end page
// [RESET: page_start = 0, page_end = 7]
////////////////////////////////////////////////////////////////////////

void SCR_set_page(unsigned char page_start, unsigned char page_end) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_PAGES;
    screen_buff[3] = page_start;
    screen_buff[4] = page_end;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 5, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


#ifdef SSD1306_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set GDDRAM page start address (L), page mode
// ---------------------------------------------------------------------
// Only for page addressing mode
//
// page_start  = 0..7, start page
// [RESET = 0]
////////////////////////////////////////////////////////////////////////

void SCR_set_page_L(unsigned char page_start_L) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_START_PAGE_CMD | (page_start_L & b_0000_0111);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


 /*******************************************************************/
 /** 4. Hardware configuration (panel resolution & layout related) **/
 /*******************************************************************/
#ifdef SSD1306_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set display start line
// ---------------------------------------------------------------------
// start_line = 0..63 [RESET = 0]
////////////////////////////////////////////////////////////////////////

void SCR_set_start_line(unsigned char start_line) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_START_LINE_CMD | start_line;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set segment re-mapping
// ---------------------------------------------------------------------
// remap = 0, switch off re-mapping [RESET]
// remap = 1, column address 127 is mapped to SEG0 (remap is ON)
////////////////////////////////////////////////////////////////////////

void SCR_set_remap(unsigned char remap) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_MAP_SEG0_CMD | (remap & b_0000_0001);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif

#ifdef SSD1306_32 // 128x32 screen
////////////////////////////////////////////////////////////////////////
// Call this function to set multiplex ratio
// ---------------------------------------------------------------------
// lines = 15..63, number of display lines - 1 [RESET = 63]
////////////////////////////////////////////////////////////////////////

static void SCR_set_mux(unsigned char lines) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_SET_MUX;
    screen_buff[3] = lines;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


#ifdef SSD1306_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set scan direction
// ---------------------------------------------------------------------
// direction = 0, scan from COM0 to number of display lines [RESET]
// direction = 1, scan from number of display lines to COM0
////////////////////////////////////////////////////////////////////////

void SCR_set_scan_direction(unsigned char direction) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_1BYTE_CMD;
    screen_buff[2] = SCR_SCAN_CMD | ((direction & b_0000_0001) << 3);
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 3, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to set display vertical offset
// ---------------------------------------------------------------------
// v_offset = -32..31 [RESET = 0]
////////////////////////////////////////////////////////////////////////

void SCR_set_v_offset(char v_offset) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_COM_V_OFFSET;
    screen_buff[3] = 64 + v_offset; // 6-bit, v_offset [-32..31]
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


////////////////////////////////////////////////////////////////////////
// Call this function to set COM pins hardware configuration
// ---------------------------------------------------------------------
// hw_com_config = 2 (SCR_CFG_NORMAL), sequential COM pin config
// hw_com_config = 18 (SCR_CFG_INTERLACED), alternative COM pin config
// [RESET = 18]
// hw_com_config = 34 (SCR_CFG_NORMAL_FLIP_LR), sequential + remap L, R
// hw_com_config = 50 (SCR_CFG_INTERLACED_FLIP_LR), alt + remap L, R
////////////////////////////////////////////////////////////////////////

void SCR_set_hw_com_cfg(unsigned char hw_com_config) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_COM_CONFIG;
    screen_buff[3] = hw_com_config;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


              /****************************************/
              /** 5. Timing & driving scheme setting **/
              /****************************************/
#ifdef SSD1306_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set display clocks/oscillator frequency
// ---------------------------------------------------------------------
// clocks = 0..255 ((SCR_F_OSC << 4) | SCR_D), frequency and divider,
// [RESET = 128].
////////////////////////////////////////////////////////////////////////

void SCR_set_freq(unsigned char clocks) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_SET_CLOCKS;
    screen_buff[3] = clocks;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}
#endif


                        /*********************/
                        /** 5a. Charge pump **/
                        /*********************/
////////////////////////////////////////////////////////////////////////
// Call this function to enable internal charge pump
// ---------------------------------------------------------------------
// [RESET = disabled]
////////////////////////////////////////////////////////////////////////

// Because of the 3.3V inline stabilizer, and no external power source
// connected to the LEDs drivers, there is no sense to disable the
// voltage pump - better to disconnect whole device from the power.

static void SCR_charge_pump_enable(void) {
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_CMD;
    screen_buff[2] = SCR_CHRG_PUMP;
    screen_buff[3] = SCR_PUMP_EN;
    TWI_Start_Transceiver_With_Data(&screen_buff[0], 4, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);
}


/************************** DATA operations ***************************/
// Data operations always increments internal video memory pointer of
// the display device, so re-send message (in case previous I2C transfer
// fails) can overwrite picture beyond defined borders. Thus, message
// importance TWI_MSG_IMP_LOW was used in drawing procedures.
////////////////////////////////////////////////////////////////////////
// Call this function to clear the part of the screen
// ---------------------------------------------------------------------
// Requires: sequential memory access.
// Zone is byte aligned, max square is 32767 bytes (screen has 1024).
// Requires: col_end >= col_start.
// Requires: page_end >= page_start.
//
// col_start  = 0..127, start column
// col_end    = 0..127, end column
// page_start = 0..7, start page
// page_end   = 0..7, end page
////////////////////////////////////////////////////////////////////////

void SCR_clear_rect(unsigned char col_start, unsigned char col_end,
                    unsigned char page_start, unsigned char page_end) {
    SCR_set_col(col_start, col_end);
    SCR_set_page(page_start, page_end);

    // Get zone square in bytes (maximum value is 1024 bytes)
    unsigned short int zone_sqr = (unsigned short int)(col_end - col_start + 1) *
                                  (unsigned short int)(page_end - page_start + 1);

    // Zeroing buffer to maximum
    unsigned char max;
    if (zone_sqr > (TWI_BUFFER_SIZE - 2)) {
        max = TWI_BUFFER_SIZE;
    } else {
        max = (unsigned char)zone_sqr + 2; // + 2 (for TWI transfer)
    }
    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_DATA;
    for (unsigned char i = 2; i < max; ++i) {
        screen_buff[i] = 0;
    }

    // If area is bigger than the buffer
    // then make number of full transmissions
    while (zone_sqr > (TWI_BUFFER_SIZE - 2)) {
            TWI_Start_Transceiver_With_Data(&screen_buff[0], TWI_BUFFER_SIZE, TWI_MSG_IMP_LOW + TWI_MSG_STOP);
            zone_sqr -= TWI_BUFFER_SIZE - 2; // Subtract transmitted pixel data
    }
    // Transmit what is left of the zone_sqr + 2 (for TWI transfer)
    TWI_Start_Transceiver_With_Data(&screen_buff[0], (unsigned char)zone_sqr + 2, TWI_MSG_IMP_LOW + TWI_MSG_STOP);
}


////////////////////////////////////////////////////////////////////////
// Call this function to display (draw) decimal number or sequence of
// pictures of the same height
// ---------------------------------------------------------------------
// Requires: sequential memory access.
// Requires: col_end >= col_start.
// Requires: font_height_px / 8  + page_start - 1 <= max page numbers (7)
//
// str[7]        = array that represents decimal number, usually 5 digits + '\0' + length,
//                 the 451 -> {4, 5, 1, 0, 0, 0, 3}.
// selected_font = 0..255, type of the font from the fonts_set_pp table (will be used for drawing)
// col_start     = 0..127, start column
// col_end       = 0..127, end column
// page_start    = 128 + 64 + 0..7; byte = YY000XXX, XXX - start page, YY - effect
// ---------------------------------------------------------------------
// After processing:
// draws string on the screen and returns width of the string in pixels
////////////////////////////////////////////////////////////////////////

unsigned char SCR_print_D (char* str, unsigned char selected_font,
             unsigned char col_start, unsigned char col_end,
             unsigned char page_start) {
    // Pointer to the font
    const unsigned char** font_pp = fonts_set_pp[selected_font];
    // Pointer to the font properties
    const unsigned char* font_prop_p = *fonts_set_pp[selected_font + 1];
    // Font spacing byte
    const unsigned char font_sp_dd_px = **fonts_set_pp[selected_font + 2];

    // Font spacing (tracking), in pixels
    const unsigned char font_sp_px = font_sp_dd_px & GLYPH_SPACE_MASK;

    unsigned char s; // Symbol position in the str[]

    // Total width of the string
    //unsigned char str_len = str[DIS_STR_NOD]; // In symbols
    unsigned char str_width_px = 0;             // In pixels
    for (s = 0; s < str[DIS_STR_NOD]; ++s) {
        str_width_px += font_sp_px;
        str_width_px += pgm_read_byte(font_prop_p + str[s]) & DRAW_WIDTH_MASK;
    }

    // Only return the string length
    if (page_start & NO_DRAW) {
        return str_width_px;
    }

    unsigned char alignment = page_start & ALIGN_RIGHT;
    page_start &= SCR_START_PAGE_MASK; // Effects bits masked
    const unsigned char page_end = page_start + ((font_sp_dd_px >> GLYPH_PAGES_BITS) & SCR_START_PAGE_MASK);

    if (alignment) {
        // Align string to the right.
        // This also sets the start/end page inside the SCR_clear_rect() call.
        // Make area left to the string blank, adjust zone/pointer.
        // String width in px should be less or equal than end column.
        unsigned char new_start_point = col_end - str_width_px + 1;
        if (new_start_point > col_start) {
            SCR_clear_rect(col_start, new_start_point - 1,
                           page_start, page_end);
            col_start = new_start_point;
        }
    }

    // Set draw zone for the glyph
    SCR_set_col(col_start, col_end);
    SCR_set_page(page_start, page_end);
    // Now buffer filled with:
    // [1] = SCR_MULTIBYTE_CMD etc.

    // Here prepare to draw the signs one by one

    //screen_buff[0] = SCR_ADDR_WR;
    screen_buff[1] = SCR_MULTIBYTE_DATA;
    // Stored height = draw height, always (no vertical symmetry)
    const unsigned char stored_height_bytes = page_end - page_start + 1; // In bytes
    // Spacing data length (W x H), in bytes, + 2 for TWI transfer
    const unsigned char sp_data_len_plus2 = font_sp_px * stored_height_bytes + 2;

    for (s = 0; s < str[DIS_STR_NOD]; ++s) {
        // Assuming the glyph has no symmetric (stored with full width)
        const unsigned char glyph_draw_width = pgm_read_byte(font_prop_p + str[s]) & DRAW_WIDTH_MASK;
        unsigned char glyph_stored_width; // In pixels
        glyph_stored_width = glyph_draw_width;
        // Glyph draw size in bytes + 2 (for TWI transfer)
        const unsigned char glyph_draw_buffer_size = glyph_draw_width * stored_height_bytes + 2;
        const unsigned char symmetric_glyph = pgm_read_byte(font_prop_p + str[s]) & H_SYMM_MASK;

        // Check if glyph is symmetric
        if (symmetric_glyph) {
            // Check parity of the columns in the glyph
            //if (glyph_stored_width & b_0000_0001) {
            if (glyph_draw_width & b_0000_0001) {
                glyph_stored_width += 1;
            }
            // Symmetric glyph, only half is stored
            glyph_stored_width /= 2;
        }
        // Stored glyph length (always less than buffer)
        const unsigned char stored_data_length = stored_height_bytes * glyph_stored_width; // In bytes

        unsigned char b; // Byte position inside the glyph

        // Add spacing before each glyph and transfer it separately
        for (b = 2; b < sp_data_len_plus2; ++b) {
            screen_buff[b] = 0;
        }
        TWI_Start_Transceiver_With_Data(&screen_buff[0], sp_data_len_plus2, TWI_MSG_IMP_LOW + TWI_MSG_STOP);

        // Fill the buffer with glyph data
        for (b = 0; b < stored_data_length; ++b) {
            // First 2 elements in screen_buff is used by TWI transfer
            screen_buff[b + 2] = pgm_read_byte(*(font_pp + str[s]) + b);
        }

        // Check if glyph is symmetric
        if (symmetric_glyph) {
            // Draw next columns mirrored
            unsigned char j = b + 2; // N-th column in the screen_buff
            // Check parity of the columns in the drawn glyph
            if (glyph_draw_width & b_0000_0001) {
                // Odd number of columns in the glyph, skip the middle
                b -= stored_height_bytes;
            }
            b -= stored_height_bytes; // Shift one column to the left
            b += 2; // For TWI transfer
            do {
                // Copy 1 column
                for (unsigned char k = 0; k < stored_height_bytes; ++k) {
                        screen_buff[j++] = screen_buff[b++];
                }
                b -= stored_height_bytes * 2; // Shift 2 more columns to the left
            // Do until full glyph length in bytes will be reached (+ 2 for TWI transfer)
            } while (j < glyph_draw_buffer_size);
        }

        // Each glyph byte size cannot exceed the size of TWI_BUFFER_SIZE - 2
        TWI_Start_Transceiver_With_Data(&screen_buff[0], glyph_draw_buffer_size, TWI_MSG_IMP_LOW + TWI_MSG_STOP);
    } // whole str[] drawn

    // Make area right to the string blank
    unsigned char blank_area_start_point = col_start + str_width_px;
    if (col_end > blank_area_start_point - 1) {
            SCR_clear_rect(blank_area_start_point, col_end,
                           page_start, page_end);
    }

    return str_width_px;
}


/****************************** General *******************************/

////////////////////////////////////////////////////////////////////////
// Call this function to set display defaults after the power on.
// ---------------------------------------------------------------------
// Add here settings that should alternate the after RESET conditions
////////////////////////////////////////////////////////////////////////

void SCR_wakeup(void) {
#ifdef SSD1306_32
    SCR_set_mux(SCR_MUX_DEFAULT);         // 128x32 screen
    SCR_set_hw_com_cfg(SCR_CFG_NORMAL);   // Matrix has sequential COM pins
                                          // layout (wires on the right only).
#else
    SCR_set_hw_com_cfg(SCR_CFG_INTERLACED);  // Matrix has alternative COM pins
                                             // layout (wires on both sides).
#endif
    SCR_set_mem_mode(SCR_MEM_VERT_MODE);  // Vertical GDDRAM access
    SCR_clear_rect(0, SCR_SEGMENTS - 1,   // Clear GDDRAM
                   0, SCR_GDDRAM_PAGES - 1);
    SCR_charge_pump_enable();             // Use internal power source
    SCR_set_panel_power_on(1);            // Start the panel
}
