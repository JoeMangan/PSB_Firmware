/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer))
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE


#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
HAL_StatusTypeDef status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* Buffer used for transmission */
uint8_t aTxBuffer[4];

/* Buffer used for reception */
uint8_t aRxBuffer[4];

// Deleteme
#define INCREMENT_DELAY 500
#define PAUSE_DELAY 1000

// I2C related definitions
#define I2C_TIMEOUT_DURATION 10 // Not sure of the units of this - could be ms
#define I2C_TX_ATTEMPT_PERIOD 100 // ms
#define I2C_TX_MAX_ATTEMPTS 5
#define I2C_RX_ATTEMPT_PERIOD 100 // ms
#define I2C_RX_MAX_ATTEMPTS 5



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


// Function prototypes
void ht_enable_on(void);
void ht_enable_off(void);

void cea_enable_on(void);
void cea_enable_off(void);
HAL_StatusTypeDef cea_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);
HAL_StatusTypeDef cea_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef cea_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX);

void ucd_enable_on(void);
void ucd_enable_off(void);
HAL_StatusTypeDef ucd_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);

// Deleteme
uint8_t i = 0x00;
uint16_t max_value = 70;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  aRxBuffer[0]=0x00;
  aRxBuffer[1]=0x00;
  aRxBuffer[2]=0x00;
  aRxBuffer[3]=0x00;
  aTxBuffer[0]=0xAA;
  aTxBuffer[1]=0xBB;
  aTxBuffer[2]=0xCC;
  aTxBuffer[3]=0xDD;

  //HAL_I2C_MspInit();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Enable the Listen mode with Interrupts for the I2C:
  // The Listen Mode will wait for an I2C Event to occur
  // and will be treated in the Interrupt Service Routine
  // of the I2C.
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1)
  {
	  if (Xfer_Complete ==1)
	  {
		HAL_Delay(1);
		/*##- Put I2C peripheral in listen mode process ###########################*/
		if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
		{
		  /* Transfer error in reception process */
		  Error_Handler();
		}
		  Xfer_Complete =0;
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    ucd_enable_on();

	uint8_t command[3] = {UCD_DAC_VBIAS_INDEX,
						  0x10,
						  0x00};

	HAL_StatusTypeDef status = ucd_i2c_write(ADDR_UCD_DAC, &command[0], 3);


  }
	/*
	* CEA TEST
	*/
	  /*
	cea_enable_on();
	HAL_Delay(100);

	uint8_t command[2] = {0x00, 0x00};
	uint8_t data[2]    = {0x00, 0x00};
	HAL_StatusTypeDef status = cea_i2c_write_read(ADDR_CEA_DIGIPOT, &command[0], 2, &data[0], 1);

    HAL_Delay(100);

    ht_enable_on();
    HAL_Delay(100);



	// Loop around and get/set
	while(i < max_value)
	{

		HAL_StatusTypeDef status = cea_i2c_read(ADDR_CEA_DIGIPOT, &data[0], 1);

		command[1] = i;
		status = cea_i2c_write_read(ADDR_CEA_DIGIPOT, &command[0], 2, &data[0], 1);

		// Increment
		i++;

		// 500ms delay
		HAL_Delay(INCREMENT_DELAY);
	}

	i--;
	HAL_Delay(PAUSE_DELAY);

	*/
  /* USER CODE END 3 */
//}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909BEB;
  hi2c1.Init.OwnAddress1 = ADDR_I2C_SLV;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909BEB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00909BEB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENABLE_HT_Pin|LD4_Pin|ENABLE_1_FPGA_Pin|ENABLE_2_IJC_Pin
                          |ENABLE_3_CEA_Pin|ENABLE_4_Pin|ENABLE_5_UCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_HT_Pin LD4_Pin ENABLE_1_FPGA_Pin ENABLE_2_IJC_Pin
                           ENABLE_3_CEA_Pin ENABLE_4_Pin ENABLE_5_UCD_Pin */
  GPIO_InitStruct.Pin = ENABLE_HT_Pin|LD4_Pin|ENABLE_1_FPGA_Pin|ENABLE_2_IJC_Pin
                          |ENABLE_3_CEA_Pin|ENABLE_4_Pin|ENABLE_5_UCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void ht_enable_on(void)
{
  HAL_GPIO_WritePin(ENABLE_HT_GPIO_Port, ENABLE_HT_Pin, GPIO_PIN_SET);
  //GPIOB->BSRR = 0x00000080;   // CEA enable high
}

void ht_enable_off(void)
{
  HAL_GPIO_WritePin(ENABLE_HT_GPIO_Port, ENABLE_HT_Pin, GPIO_PIN_RESET);
  //GPIOB->BSRR = 0x00800000;   // CEA enable low
}


//************************************
//            UCD PSB
//************************************

// Board enable functions
void ucd_enable_on(void)
{
  HAL_GPIO_WritePin(ENABLE_5_UCD_GPIO_Port, ENABLE_5_UCD_Pin, GPIO_PIN_SET);
}

void ucd_enable_off(void)
{
  HAL_GPIO_WritePin(ENABLE_5_UCD_GPIO_Port, ENABLE_5_UCD_Pin, GPIO_PIN_RESET);
}


HAL_StatusTypeDef ucd_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

//************************************
//            CEA PSB
//************************************

// Board enable functions
void cea_enable_on(void)
{
  HAL_GPIO_WritePin(ENABLE_3_CEA_GPIO_Port, ENABLE_3_CEA_Pin, GPIO_PIN_SET);
  //GPIOB->BSRR = 0x00000080;   // CEA enable high
}

void cea_enable_off(void)
{
  HAL_GPIO_WritePin(ENABLE_3_CEA_GPIO_Port, ENABLE_3_CEA_Pin, GPIO_PIN_RESET);
  //GPIOB->BSRR = 0x00800000;   // CEA enable low
}

HAL_StatusTypeDef cea_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef cea_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX)
{
	// Read bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c3, dev_addr, in_ptr, countRX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef cea_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX)
{
	// The status used to indicate success/error
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t tx_attemps = I2C_TX_MAX_ATTEMPTS;
	uint8_t rx_attemps = I2C_RX_MAX_ATTEMPTS;

	do
	{
		// Write operation
		if(tx_attemps < I2C_TX_MAX_ATTEMPTS){HAL_Delay(I2C_TX_ATTEMPT_PERIOD);} // Delay if re-attempting I2C Operation
		status = cea_i2c_write(dev_addr, out_ptr, countTX);						// Perform I2C operation
		tx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (tx_attemps > 0));						    // Check the I2C operation status

	if(status == HAL_ERROR){return(status);}

	do
	{
		// Read operation
		status =  cea_i2c_read(dev_addr, in_ptr, countRX); 						// Delay if re-attempting I2C Operation
		if(rx_attemps < I2C_RX_MAX_ATTEMPTS){HAL_Delay(I2C_RX_ATTEMPT_PERIOD);}	// Perform I2C operation
		rx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (rx_attemps > 0));							// Check the I2C operation status

	return(status);
}




/* USER CODE BEGIN 4 */
/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED4: Transfer in transmission process is correct */

  Xfer_Complete = 1;
  aTxBuffer[0]++;
  aTxBuffer[1]++;
  aTxBuffer[2]++;
  aTxBuffer[3]++;

}


/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED4: Transfer in reception process is correct */

  Xfer_Complete = 1;
  aRxBuffer[0]=0x00;
  aRxBuffer[1]=0x00;
  aRxBuffer[2]=0x00;
  aRxBuffer[3]=0x00;
}



/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  Transfer_Direction = TransferDirection;
  if (Transfer_Direction != 0)
  {
     /*##- Start the transmission process #####################################*/
  /* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
  if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)

    {
    /* Transfer error in transmission process */
    Error_Handler();
  }

  }
  else
  {

      /*##- Put I2C peripheral in reception process ###########################*/
  if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)aRxBuffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
    {
    /* Transfer error in reception process */
    Error_Handler();
  }

  }

}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave doesn't acknowledge its address, Master restarts communication.
    * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
