/*
 * max9611.h
 *
 *  Created on: Aug 14, 2023
 *      Author: jm
 */

#ifndef INC_MAX9611_H_
#define INC_MAX9611_H_


// Internal register addresses
// -----------------------------------
#define CSA_DATA_BYTE_MSB     0x00 //
#define CSA_DATA_BYTE_LSB     0x01 //
#define RSP_DATA_BYTE_MSB     0x02 //
#define RSP_DATA_BYTE_LSB     0x03 //
#define OUT_DATA_BYTE_MSB     0x04 //
#define OUT_DATA_BYTE_LSB     0x05 //
#define SET_DATA_BYTE_MSB     0x06 //
#define SET_DATA_BYTE_LSB     0x07 //
#define TEMP_DATA_BYTE_MSB    0x08 //
#define TEMP_DATA_BYTE_LSB    0x09 //
#define CONTROL_REGISTER_1    0x0A //
#define CONTROL_REGISTER_2    0x0B //

// Control register 1 configuration
// -----------------------------------
// MUX 2, 1, 0 settings
#define CHANNEL_A_GAIN_1x_BIN         0b00000000 // Read current-sense amplifier output from ADC, gain = 1x
#define CHANNEL_A_GAIN_4x_BIN         0b00000001 // Read current-sense amplifier output from ADC, gain = 4x
#define CHANNEL_A_GAIN_8x_BIN         0b00000010 // Read current-sense amplifier output from ADC, gain = 8x
#define CHANNEL_B_ADV_VOLT_BIN        0b00000011 // Read average voltage of RS+ (input common-mode voltage) from ADC
#define CHANNEL_C_OUT_VOLT_BIN        0b00000100 // Read voltage of OUT from ADC
#define CHANNEL_D_SET_VOLT_BIN        0b00000101 // Read voltage of SET from ADC
#define CHANNEL_E_DIE_TEMP_BIN        0b00000110 // Read internal die temperature from ADC
#define CHANNEL_FAST_READ_BIN         0b00000111 // Read all channels in fast-read mode, sequentially every 2ms. Uses last gain setting
// Shutdown mode
#define SHDN_NORMAL_OPERATION_BIN     0b00000000 // 0 = Normal operation
#define SHDN_SHUTDOWN_MODE_BIN        0b00001000 // 1 = Shutdown mode
// Latch/retry functionality
#define LD_NORMAL_OPERATION_BIN       0b00000000 // 0 = Normal operation
#define LD_RESET_BIN                  0b00010000 // 1 = Reset if comparator is latched due to MODE = 111. This bit is automatically reset after a 1 is written
// Mode for op amp/comparator
#define AMP_COMP_NORMAL_OPERATION_BIN 0b00000000 // Normal operation for op amp/comparator
#define AMP_COMP_COMPARITOR_MODE_BIN  0b11100000 // Comparator mode
#define AMP_COMP_OPAMP_MODE_BIN       0b01100000 // Op-amp mode

// MUX 2, 1, 0 settings
#define CHANNEL_A_GAIN_1x_DEC         0 // Read current-sense amplifier output from ADC, gain = 1x
#define CHANNEL_A_GAIN_4x_DEC         1 // Read current-sense amplifier output from ADC, gain = 4x
#define CHANNEL_A_GAIN_8x_DEC         2 // Read current-sense amplifier output from ADC, gain = 8x
#define CHANNEL_B_ADV_VOLT_DEC        3 // Read average voltage of RS+ (input common-mode voltage) from ADC
#define CHANNEL_C_OUT_VOLT_DEC        4 // Read voltage of OUT from ADC
#define CHANNEL_D_SET_VOLT_DEC        5 // Read voltage of SET from ADC
#define CHANNEL_E_DIE_TEMP_DEC        6 // Read internal die temperature from ADC
#define CHANNEL_FAST_READ_DEC         7 // Read all channels in fast-read mode, sequentially every 2ms. Uses last gain setting
// Shutdown mode
#define SHDN_NORMAL_OPERATION_DEC     0 // 0 = Normal operation
#define SHDN_SHUTDOWN_MODE_DEC        1 // 1 = Shutdown mode
// Latch/retry functionality
#define LD_NORMAL_OPERATION_DEC       0 // 0 = Normal operation
#define LD_RESET_DEC                  1 // 1 = Reset if comparator is latched due to MODE = 111. This bit is automatically reset after a 1 is written
// Mode for op amp/comparator
#define AMP_COMP_NORMAL_OPERATION_DEC 0 // Normal operation for op amp/comparator
#define AMP_COMP_COMPARITOR_MODE_DEC  7 // Comparator mode
#define AMP_COMP_OPAMP_MODE_DEC       3 // Op-amp mode


// Control register 2 configuration
// -----------------------------------

// Watchdog delay time
#define DTIM_1_MS_BITS_BIN                 0b00000000 // 0 = 1ms
#define DTIM_100_US_BITS_BIN               0b00001000 // 1 = 100μs
// Watchdog retry delay time
#define RTIM_50_MS_BITS_BIN                0b00000000 // 0 = 50ms
#define RTIM_10_MS_BITS_BIN                0b00000100 // 1 = 10ms

// Watchdog delay time
#define DTIM_1_MS_BITS_DEC                 0 // 0 = 1ms
#define DTIM_100_US_BITS_DEC               1 // 1 = 100μs
// Watchdog retry delay time
#define RTIM_50_MS_BITS_DEC                0 // 0 = 50ms
#define RTIM_10_MS_BITS_DEC                1 // 1 = 10ms



typedef union {
    // A union template for defining the command byte structure for CTRL Reg 1
    // ----------------------------------------------------------------------
    struct{                    // The bit field structure for the command
        uint8_t MUX   :3;      // Channel setting
        uint8_t SHDN  :1;      // SHDN state
        uint8_t LR    :1;      // Latch setting
        uint8_t MODE  :3;      // OPAMP/COMPARITOR mode
    } bits;

    uint8_t byte;              // The full byte
} _ctrl_reg_1_byte;

typedef union {
    // A union template for defining the command byte structure for CTRL Reg 2
    // ----------------------------------------------------------------------
    struct{                    // The bit field structure for the command
        uint8_t RES_1 :2;      // Reserved
        uint8_t RTIM  :1;      // Watchdog delay time
        uint8_t DTIM  :1;      // Watchdog retry delay time
        uint8_t RES_2 :4;      // Reserved
    } bits;
    uint8_t byte;              // The full byte
} _ctrl_reg_2_byte;

typedef union {
    // A union template for defining the byte structure of recieved databytes
    // ----------------------------------------------------------------------
    struct{
        uint8_t lsb_byte:8; // The least significant byte
        uint8_t msb_byte:8; // The most significant byte
    } bytes;
    uint16_t word;          // The full word
} _data_byte;

// A structure for storing the measurement data
struct max6811_registers {
    // Read reg values
    _data_byte csa_data_byte;       // The recieved data over i2c
    _data_byte rsp_data_byte;       // The recieved data over i2c
    _data_byte out_data_byte;       // The recieved data over i2c
    _data_byte set_data_byte;       // The recieved data over i2c
    _data_byte temp_data_byte;      // The recieved data over i2c
    _ctrl_reg_1_byte ctrl1_data_byte;     // The recieved data over i2c
    _ctrl_reg_2_byte ctrl2_data_byte;     // The recieved data over i2c

    // Write reg values
    _ctrl_reg_1_byte ctrl_reg_1;    // Byte storing the configuration for control reg 1
    _ctrl_reg_2_byte ctrl_reg_2;    // Byte storing the configuration for control reg 2
};


#endif /* INC_MAX9611_H_ */
