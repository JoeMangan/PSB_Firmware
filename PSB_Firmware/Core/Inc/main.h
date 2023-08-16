/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADDR_CEA_DIGIPOT 0b0101100
#define ADDR_I2C_SLV 0x32
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define ENABLE_HT_Pin GPIO_PIN_12
#define ENABLE_HT_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_13
#define LD4_GPIO_Port GPIOB
#define TIMING_PIN_Pin GPIO_PIN_6
#define TIMING_PIN_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENABLE_1_FPGA_Pin GPIO_PIN_5
#define ENABLE_1_FPGA_GPIO_Port GPIOB
#define ENABLE_2_IJC_Pin GPIO_PIN_6
#define ENABLE_2_IJC_GPIO_Port GPIOB
#define ENABLE_3_CEA_Pin GPIO_PIN_7
#define ENABLE_3_CEA_GPIO_Port GPIOB
#define ENABLE_4_Pin GPIO_PIN_8
#define ENABLE_4_GPIO_Port GPIOB
#define ENABLE_5_UCD_Pin GPIO_PIN_9
#define ENABLE_5_UCD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Revise the UCD stuff
#define ADDR_UCD_MAX9611_AVDD   0b11100100
#define ADDR_UCD_MAX9611_DVDD   0b11100010
// ----------------------------------------
#define ADDR_UCD_DAC     		0x9E
#define UCD_DAC_VBIAS_INDEX     0x10
#define UCD_DAC_MBIAS_1_INDEX   0x12
#define UCD_DAC_MBIAS_2_INDEX   0x14

// Dev addrs on the IJC board
#define ADDR_IJC_MAX9611_1_5    	  0b11100000    // The 1.5V supply (U11 on the schematic)
#define ADDR_IJC_MAX9611_2      	  0b11100010    // The -2V supply  (U12 on the schematic)
#define ADDR_IJC_MAX9611_HV_CURRENT   0b11100110    // The HV current  (U13 on the schematic)
#define ADDR_IJC_MAX9611_HV_VOLTAGE   0b11100100    // The HV voltage  (U14 on the schematic)
#define ADDR_IJC_DIGIPOT			  0b01011000	// The IJC Digipot (U5  on the schematic)


// I2C slv command list
// --------------------------------------
#define CMD_READ							0x01
#define CMD_WRITE                           0x00
#define CMD_FAIL_OP_RESP                    0xFFFF
#define CMD_FAIL_OP_INPROG_RESP             0xFFFE
#define CMD_SUCCESS_RESP                    0x0001
// ----------------------------------------------
// UCD Detector Commands
#define CMD_UCD_ENABLE 						0x80
// xxxxxxxxxx
#define CMD_UCD_AVDD_VOLTAGE                0x86
#define CMD_UCD_AVDD_CURRENT                0x87
// ----------------------------------------------
// IJC Detector Commands
#define CMD_IJC_ENABLE 						0x60
// xxxxxxxxxx
#define CMD_IJC_HV_VOLTAGE 			        0x62
#define CMD_IJC_HV_CURRENT 			        0x63
// xxxxxxxxxx
#define CMD_IJC_TARGET_HV_VOLTAGE 			0x64
#define CMD_IJC_1_5_VOLTAGE 			    0x65
#define CMD_IJC_1_5_CURRENT 			    0x66
#define CMD_IJC_2_VOLTAGE 			        0x67
#define CMD_IJC_2_CURRENT 			        0x68





// A union struct to hold the I2C slv RX
typedef union{
	struct
	{
		uint8_t data_byte_lsb     :8;
		uint8_t data_byte_msb     :8;
		uint8_t rw_state	      :8;
		uint8_t cmd			      :8;
	} bytes;
	uint32_t data;
} _i2c_slv_rx;


// A union struct to hold the I2C slv TX
typedef union{
	struct
	{
	  uint8_t byte_2   :8;
	  uint8_t byte_1   :8;
	} bytes;
	uint16_t data;
} _i2c_slv_tx;


typedef struct
{
	bool     ramp_flag;
	uint8_t  hv_digipot_value;
	uint16_t hv_targate_value;
	uint16_t board_enable_state;
} _detector;


typedef enum max6911_ctrl_ENUM
{
	INIT_WITH_CURRENT_GAIN_8		= 0x00,
	FASTREAD_NORMAL_OPERATION	    = 0x01
} _max6911_ctrl;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
