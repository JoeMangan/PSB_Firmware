/*
 * psb.c
 *
 *  Created on: Aug 14, 2023
 *      Author: jm
 */

#include "main.h"
#include "psb.h"

//_detector ucd_detector;


/*
bool i2c_slv_cmd_rx_tx_handle(void)
{
	bool status = EXIT_SUCCESS;

	// SOME FUNCTION HERE TO HANDLE RX/TX
    switch(i2c_slv_rx.bytes.cmd)
    {
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	case(CMD_UCD_ENABLE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				ucd_detector.board_enable_state = (uint16_t)ucd_board_enable_get(); // Read the state enable pin
				i2c_slv_tx.data = ucd_detector.board_enable_state; 					// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				// Write the state of the enable pin
				if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_SET)
				{
					ucd_board_enable_set(GPIO_PIN_SET);								// Set the state of the pin
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_RESET)
				{
					ucd_board_enable_set(GPIO_PIN_RESET);                           // Set the state of the pin
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				i2c_slv_tx.data = CMD_FAIL_OP_RESP;
				status =  EXIT_FAILURE;
				return(status);
			}
			break;
		}
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	case(CMD_UCD_AVDD_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{


				ucd_detector.board_enable_state = (uint16_t)ucd_board_enable_get(); // Read the state enable pin
				i2c_slv_tx.data = ucd_detector.board_enable_state; 					// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				i2c_slv_tx.data = CMD_FAIL_OP_RESP;
				status =  EXIT_FAILURE;
				return(status);
			}
			break;
		}
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	default:
			break;
	}

}
*/
