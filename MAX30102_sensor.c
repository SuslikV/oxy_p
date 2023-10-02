/*
 * Controls for the optical sensor based on the MAX30102
 * IR, RED LED heart monitor and temperature sensor.
 * Communications via I2C interface (TWI_Master.c file, M20230827).
 *
 * Version: M20230827
 */

#include "TWI_Master.h"      // I2C master driver
#include "MAX30102_sensor.h" // Sensor specific definitions

// Sensor buffer. Not less than 5 bytes. Up to 255 bytes (<= TWI buffer size).
// By default set to TWI buffer size (130 bytes).
extern unsigned char sens_buff[];


//!!!!!!!!!!!!!!!!!!!!!!!!!!    Commands    !!!!!!!!!!!!!!!!!!!!!!!!!!//

                        //!!!!!!!!!!!!!!!!!!//
                        //!   1. Status    !//
                        //!!!!!!!!!!!!!!!!!!//

#ifdef MAX30102_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to get status of the sensor (clears interrupts).
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register value will be temporary stored.
//             Array capacity is 4 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - Interrupt Status 1 register value;
//             [2] - Interrupt Status 2 register value.
//
// All sensor's interrupts cleared.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_status(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = 0; // Interrupt Status 1, PWR_RDY, etc.
    //sens_buff[2] = 0; // Interrupt Status 2, DIE_TEMP_RDY
    sens_buff[2 + 1] = SENS_REG_INT_STAT1;
    return SENS_get();
}
#endif


#ifdef MAX30102_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to get allowed interrupts.
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register value will be temporary stored.
//             Array capacity is 4 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - Interrupt Enable 1 register value;
//             [2] - Interrupt Enable 2 register value.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_int_en(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = 0; // Interrupt Enable 1, ALC_OVF_EN, etc.
    //sens_buff[2] = 0; // Interrupt Enable 2, DIE_TEMP_RDY_EN
    sens_buff[2 + 1] = SENS_REG_INT_ENBL1;
    return SENS_get();
}


////////////////////////////////////////////////////////////////////////
// Call this function to set allowed interrupts.
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register value will be temporary stored.
//             Array capacity is 4 bytes minimum.
//             [1] - Interrupt Enable 1 register value;
//             [2] - Interrupt Enable 2 register value.
////////////////////////////////////////////////////////////////////////

void SENS_set_int_en(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = xx; // Interrupt Enable 1 value
    //sens_buff[2] = xx; // Interrupt Enable 2 value
    sens_buff[2 + 1] = SENS_REG_INT_ENBL1;
    SENS_set();
}
#endif


                        //!!!!!!!!!!!!!!!!!!//
                        //!    2. FIFO     !//
                        //!!!!!!!!!!!!!!!!!!//

////////////////////////////////////////////////////////////////////////
// Call this function to get FIFO all pointers
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where FIFO pointers will be
//             temporary stored.
//             Array capacity is 5 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - FIFO write pointer (FIFO_WR_PTR);
//             [2] - FIFO number of lost samples due to buffer
//                   filled up, i.e. overflow counter (OVF_COUNTER);
//             [3] - FIFO read pointer (FIFO_RD_PTR).
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_FIFO_pointers(void) {
    sens_buff[0] = 3;
    //sens_buff[1] = 0; // FIFO_WR_PTR
    //sens_buff[2] = 0; // OVF_COUNTER
    //sens_buff[3] = 0; // FIFO_RD_PTR
    sens_buff[3 + 1] = SENS_REG_FIFO_WR_PTR;
    return SENS_get();
}


#ifdef MAX30102_EXT_02
////////////////////////////////////////////////////////////////////////
// Call this function to set FIFO all pointers
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where FIFO pointers
//             will be temporary stored.
//             Array capacity is 5 bytes minimum.
//             [1] - FIFO write pointer (FIFO_WR_PTR);
//             [2] - FIFO number of lost samples due to buffer
//                   filled up, i.e. overflow counter (OVF_COUNTER);
//             [3] - FIFO read pointer (FIFO_RD_PTR).
////////////////////////////////////////////////////////////////////////

void SENS_set_FIFO_pointers(void) {
    sens_buff[0] = 3;
    //sens_buff[1] = xx; // FIFO_WR_PTR
    //sens_buff[2] = xx; // OVF_COUNTER
    //sens_buff[3] = xx; // FIFO_RD_PTR
    sens_buff[3 + 1] = SENS_REG_FIFO_WR_PTR;
    SENS_set();
}



                     //!!!!!!!!!!!!!!!!!!!!!!!!//
                     //!   3. Configuration   !//
                     //!!!!!!!!!!!!!!!!!!!!!!!!//

////////////////////////////////////////////////////////////////////////
// Call this function to set the FIFO configuration
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register will be temporary stored. Array capacity is
//             3 bytes minimum.
//             [1] - FIFO Configuration for the sensor.
// ---------------------------------------------------------------------
// After processing:
// Sets new FIFO configuration
////////////////////////////////////////////////////////////////////////

void SENS_set_FIFO_cfg(void) {
    sens_buff[0] = 1;
    //sens_buff[1] = xx; // FIFO Configuration
    sens_buff[1 + 1] = SENS_REG_FIFO_CFG;
    SENS_set();
}


////////////////////////////////////////////////////////////////////////
// Call this function to get the FIFO configuration
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register will be temporary stored.
//             Array capacity is 3 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [1] - FIFO Configuration.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_FIFO_cfg(void) {
    sens_buff[0] = 1;
    //sens_buff[1] = 0; // FIFO Configuration
    sens_buff[1 + 1] = SENS_REG_FIFO_CFG;
    return SENS_get();
}


////////////////////////////////////////////////////////////////////////
// Call this function to set the sensor mode (IR/Red LEDs)
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register will be temporary stored.
//             Array capacity is 3 bytes minimum.
//             [1] - Mode Configuration for the sensor.
// ---------------------------------------------------------------------
// After processing:
// Sets new mode // Don't forget to resets the FIFO pointers to 0.
////////////////////////////////////////////////////////////////////////

void SENS_set_mode(void) {
    sens_buff[0] = 1;
    //sens_buff[1] = xx; // Mode Configuration
    sens_buff[1 + 1] = SENS_REG_MODE_CFG;
    SENS_set();
}
#endif


////////////////////////////////////////////////////////////////////////
// Call this function to set ADC conversion settings
// ---------------------------------------------------------------------
// conv_data = array of unsigned char where configuration
//             register will be temporary stored.
//             Array capacity is 3 bytes minimum.
//             [1] - SpO2 Configuration (ADC conversion settings:
//                   ADC full scale, sample rate, ADC bitness).
////////////////////////////////////////////////////////////////////////

void SENS_set_conversion(void) {
    sens_buff[0] = 1;
    //sens_buff[1] = xx; // SpO2 Configuration
    sens_buff[1 + 1] = SENS_REG_SPO2_CFG;
    SENS_set();
}


////////////////////////////////////////////////////////////////////////
// Call this function to set LED current
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration register
//             will be temporary stored.
//             Array capacity is 4 bytes minimum.
//             [1] - LED1 Pulse Amplitude (current = value * 0.2 mA);
//             [2] - LED2 Pulse Amplitude (current = value * 0.2 mA).
////////////////////////////////////////////////////////////////////////

void SENS_set_LED_current(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = xx; // LED1_PA;
    //sens_buff[2] = xx; // LED2_PA;
    sens_buff[2 + 1] = SENS_REG_LED1_PA;
    SENS_set();
}


                    //!!!!!!!!!!!!!!!!!!!!!!!!!//
                    //!  4. Die Temperature   !//
                    //!!!!!!!!!!!!!!!!!!!!!!!!!//

#ifdef MAX30102_EXT_01
////////////////////////////////////////////////////////////////////////
// Call this function to start die temperature measurement
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register will be temporary stored.
//             Array capacity is 3 bytes minimum.
////////////////////////////////////////////////////////////////////////

void SENS_set_die_tmpr_en(void) {
    sens_buff[0] = 1;
    sens_buff[1] = (1 << SENS_TMPR_EN_BIT);
    sens_buff[1 + 1] = SENS_REG_DIE_TMPR_CFG;
    SENS_set();
}


////////////////////////////////////////////////////////////////////////
// Call this function to know if the die temperature measurement is
// still active.
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where configuration
//             register value will be temporary stored.
//             Array capacity is 3 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - Die Temperature Config register value.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_die_tmpr_cfg(void) {
    sens_buff[0] = 1;
    //sens_buff[1] = 0; // TEMP_EN
    sens_buff[1 + 1] = SENS_REG_DIE_TMPR_CFG;
    return SENS_get();
}


////////////////////////////////////////////////////////////////////////
// Call this function to get the die temperature
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where integer and
//             fractional value of the temperature will be temporary
//             stored.
//             Array capacity 4 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address,
//             [1] - Die Temp Integer (TINT) value,
//             [2] - Die Temp Fraction (TFRAC) value.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_die_tmpr_value(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = 0; // TINT
    //sens_buff[2] = 0; // TFRAC
    sens_buff[2 + 1] = SENS_REG_DIE_TMPR_INT_VAL;
    return SENS_get();
}
#endif

                        //!!!!!!!!!!!!!!!!!!//
                        //!   5. Part ID   !//
                        //!!!!!!!!!!!!!!!!!!//

////////////////////////////////////////////////////////////////////////
// Call this function to get sensor revision and part ID
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where revision and
//             part ID will be stored.
//             Array capacity is 4 bytes minimum.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - sensor Revision ID;
//             [2] - sensor Part ID.
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_rev_id(void) {
    sens_buff[0] = 2;
    //sens_buff[1] = 0; // Revision ID
    //sens_buff[2] = 0; // Part ID
    sens_buff[2 + 1] = SENS_REG_REV_ID;
    return SENS_get();
}


//!!!!!!!!!!!!!!!!!!!!!!    DATA operations    !!!!!!!!!!!!!!!!!!!!!!!//

////////////////////////////////////////////////////////////////////////
// Call this function to get N-number of bytes from FIFO
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where FIFO sample
//             will be temporary stored.
//             Array capacity is 3 bytes minimum.
//             [1] - contains number of bytes to read from the FIFO;
//                   Max: 254 bytes.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - 1 byte, LED?[23:16];
//             [2] - 2 byte, LED?[15:8];
//             [3] - 3 byte, LED?[7:0];
//             ... - N times repeat;
//             [N - 2] - 1 byte, LED?[23:16];
//             [N - 1] - 2 byte, LED?[15:8];
//             [N - 0] - 3 byte, LED?[7:0];
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get_FIFO_N_bytes(void) {
    sens_buff[0] = sens_buff[1];
    //sens_buff[1] = 0; // LED?[23:16]
    //sens_buff[2] = 0; // LED?[15:8]
    //sens_buff[3] = 0; // LED?[7:0]
    //  ...
    sens_buff[sens_buff[1] + 1] = SENS_REG_FIFO_DATA;
    return SENS_get();
}


////////////////////////////////////////////////////////////////////////
// Call this function to get number of bytes starting from the register.
// Waits to complete the transfer.
// ---------------------------------------------------------------------
// sens_buff = array of unsigned char where parameters
//             will be temporary stored.
//             Array capacity is 3 bytes minimum.
//             [0] - contains number of parameters (n),
//                   max(n) = TWI_BUFFER_SIZE - 2;
//             [1] - storage for parameter 1;
//             [2] - storage for parameter 2;
//                    ...
//             [n] - storage for parameter n;
//             [n + 1] - register.
// ---------------------------------------------------------------------
// After processing:
// sens_buff = array of unsigned char that contains updated data.
//             [0] - I2C address;
//             [1] - parameter 1;
//             [2] - parameter 2;
//                  ...
//             [n] - parameter n;
//
// Returns result of the last transfer
////////////////////////////////////////////////////////////////////////

unsigned char SENS_get(void) {
    unsigned char n_plus1;
    n_plus1 = sens_buff[0] + 1;
    sens_buff[1] = sens_buff[n_plus1]; // Register
    sens_buff[0] = SENS_ADDR_WR;
    TWI_Start_Transceiver_With_Data(&sens_buff[0], 2, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);

    if (!TWI_Complete_Last_Transfer(IGNORE_STOP)) {
        return FALSE; // Last transmission failed
    };

    sens_buff[0] = SENS_ADDR_R;
    //sens_buff[1] = 0; // parameter 1;
    //sens_buff[2] = 0; // parameter 2;
    //  ...
    //sens_buff[n] = 0; // parameter n;

    // Here message shouldn't be re-send if last read transfer fails,
    // because internal pointer of the device may already advance the register.
    // Only whole sequence should repeat: write_from_where_to_read->read_from.
    TWI_Start_Transceiver_With_Data(&sens_buff[0], n_plus1, TWI_MSG_IMP_LOW + TWI_MSG_STOP);

    //!!! The MAX IC I have (chinese clone) requires STOP after the bus
    //!!! access. Otherwise the ADC wouldn't start (waiting for I2C).
    //!!! Should free the I2C bus as soon a possible.
    //!!! The STOP will be sent from inside of the TWI_Get_Data_From_Transceiver().

    unsigned char result;
    result = TWI_Get_Data_From_Transceiver(&sens_buff[0], n_plus1);
    return result;
}


////////////////////////////////////////////////////////////////////////
// Call this function to set number of parameters to the register(s).
// Waits to complete the transfer.
// ---------------------------------------------------------------------
// sett_data = array of unsigned char where parameters will be
//             temporary stored.
//             Array capacity is 3 bytes minimum.
//             [0] - contains number of parameters (n);
//                   max(n) = TWI_BUFFER_SIZE - 2;
//             [1] - parameter 1;
//             [2] - parameter 2;
//                    ...
//             [n] - parameter n;
//             [n + 1] - register.
////////////////////////////////////////////////////////////////////////

void SENS_set(void) {
    unsigned char n_plus1;
    n_plus1 = sens_buff[0] + 1;
    sens_buff[0] = sens_buff[n_plus1]; // Remember register
    for (unsigned char i = n_plus1; i > 0; --i){
        sens_buff[i] = sens_buff[i - 1];    // Shift values 1 cell to the end
    }

    sens_buff[0] = SENS_ADDR_WR;
    //sens_buff[1] = xx; // register;
    //sens_buff[2] = xx; // parameter 1;
    //sens_buff[3] = xx; // parameter 2;
    //  ...
    //sens_buff[n + 1] = xx; // parameter n;
    TWI_Start_Transceiver_With_Data(&sens_buff[0], n_plus1 + 1, TWI_MSG_IMP_NORMAL + TWI_MSG_STOP);

    //!!! The MAX IC I have (chinese clone) requires STOP after the bus
    //!!! access. Otherwise the ADC wouldn't start (waiting for I2C).
    //!!! Should free the I2C bus as soon a possible.

    TWI_Complete_Last_Transfer(SEND_STOP);
}
