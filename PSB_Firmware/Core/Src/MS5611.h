/*
 * MS5611.h
 *
 *  Created on: Aug 18, 2023
 *      Author: jm
 */

#ifndef SRC_MS5611_H_
#define SRC_MS5611_H_


// PROM memory data
#define RESERVED      0 // Reserved bytes
#define COEF1         1 // Coefficient 1
#define COEF2         2 // Coefficient 2
#define COED3         3 // Coefficient 3
#define COEF4         4 // Coefficient 4
#define COEF5         5 // Coefficient 5
#define COEF6         6 // Coefficient 6
#define CRC_B         7 // CRC bits [bits 2-0]
// ------------------------------------------
#define SENST1        1 // Pressure sensitivity | SENST1
#define OFFT1         2 // Pressure offset | OFFT1
#define TCS           3 // Temperature coefficient of pressure sensitivity | TCS
#define TCO           4 // Temperature coefficient of pressure offset | TCO
#define TREF          5 // Reference temperature | TREF
#define TEMPSENS      6 // Temperature coefficient of the temperature | TEMPSENS

// Oversampling Ratio levels
#define OSR_256       0 // Convesions using 256  resolution
#define OSR_512       1 // Convesions using 512  resolution
#define OSR_1024      2 // Convesions using 1024 resolution
#define OSR_2048      3 // Convesions using 2048 resolution
#define OSR_4096      4 // Convesions using 4096 resolution

// Command list for the MS5611 device
// https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
#define CMD_RESET               0x1E  // Reset command
#define CMD_CONVERT_D1_OSR_256  0x40  // ADC D1 OSR=256
#define CMD_CONVERT_D1_OSR_512  0x42  // ADC D1 OSR=512
#define CMD_CONVERT_D1_OSR_1024 0x44  // ADC D1 OSR=1024
#define CMD_CONVERT_D1_OSR_2048 0x46  // ADC D1 OSR=2048
#define CMD_CONVERT_D1_OSR_4096 0x48  // ADC D1 OSR=4096
#define CMD_CONVERT_D2_OSR_256  0x50  // ADC D2 OSR=256
#define CMD_CONVERT_D2_OSR_512  0x52  // ADC D2 OSR=512
#define CMD_CONVERT_D2_OSR_1024 0x54  // ADC D2 OSR=1024
#define CMD_CONVERT_D2_OSR_2048 0x56  // ADC D2 OSR=2048
#define CMD_CONVERT_D2_OSR_4096 0x58  // ADC D2 OSR=4096
#define CMD_ADC_READ            0x00  // ADC read command
#define CMD_PROM_READ 	        0xA0  // PROM read command (values from 0xA0 - 0xAE inclusive)


// A structure for storing the measurement data
struct meas {
	uint32_t uncomp_temp;  // Uncompensated temperature
	uint32_t uncomp_press; // Uncompensated pressure
    uint16_t prom_regs[8]; // An array to store the prom reg values


	double dT;                // Difference between actual and reference temperature
	double temp;              // Actual temperature (-40...85°C with 0.01°C resolution)
    double OFF;               // Offset at actual temperature
    double SENS;        // Sensitivity at actual temperature
    long double pressure;          // Temperature compensated pressure (10...1200mbar with 0.01mbar resolution)

    double T2;          // Temperature calculated during second order temperature compensation
    double OFF2;        // Offset calculated during second order temperature compensation
    double SENS2;       // Sensitivity calculated during second order temperature compensation
};

#endif /* SRC_MS5611_H_ */
