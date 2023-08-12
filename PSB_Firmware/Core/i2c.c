/*
 * i2c.c
 *
 *  Created on: Aug 12, 2023
 *      Author: jm
 */


/*
//#include "main.h"
//#include "stm32l4xx_hal.h"

//#include "i2c.h"


// Variables
HAL_StatusTypeDef i2c_TX_status = HAL_OK;		// Status of the TX operation
HAL_StatusTypeDef i2c_RX_status = HAL_OK;		// Status of the RX operation
// TIMEOUT DURATION
// MAX ATTAMPTS



// Functions
HAL_StatusTypeDef i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX)
{
	// Read bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, in_ptr, countRX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX)
{
	// The status used to indicate success/error
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t tx_attemps = I2C_TX_MAX_ATTEMPTS;
	uint8_t rx_attemps = I2C_RX_MAX_ATTEMPTS;

	do
	{
		// Write operation
		if(tx_attemps < I2C_TX_MAX_ATTEMPTS){HAL_Delay(I2C_TX_ATTEMPT_PERIOD);} // Delay if re-attempting I2C Operation
		status = i2c_write(dev_addr, out_ptr, countTX);						// Perform I2C operation
		tx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (tx_attemps > 0));						    // Check the I2C operation status

	if(status == HAL_ERROR){return(status);}

	do
	{
		// Read operation
		status =  i2c_read(dev_addr, in_ptr, countRX); 						// Delay if re-attempting I2C Operation
		if(rx_attemps < I2C_RX_MAX_ATTEMPTS){HAL_Delay(I2C_RX_ATTEMPT_PERIOD);}	// Perform I2C operation
		rx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (rx_attemps > 0));							// Check the I2C operation status

	return(status);
}
*/
