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
#include "max9611.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//------------------------------
// I2C Master
//------------------------------
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
//------------------------------
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Instrument related definitions
//------------------------------
// I2C related definitions
//------------------------------
// I2C Slave
//------------------------------
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXBUFFERSIZE                      2//(COUNTOF(aTxBuffer))   // Size of Tx buffer
#define RXBUFFERSIZE                      4//TXBUFFERSIZE 			// Size of Rx buffer
// I2C Master
//------------------------------
#define I2C_TIMEOUT_DURATION 10 // Not sure of the units of this - could be ms
#define I2C_TX_ATTEMPT_PERIOD 100 // ms
#define I2C_TX_MAX_ATTEMPTS 5
#define I2C_RX_ATTEMPT_PERIOD 100 // ms
#define I2C_RX_MAX_ATTEMPTS 5
//------------------------------


// Deleteme
#define INCREMENT_DELAY 500
#define PAUSE_DELAY 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef status;
_detector ucd_detector;
_detector ijc_detector;
_detector cea_detector;

struct max6811_registers max6911;             		// A structure for storing the i2c recieved data
uint8_t i2c_tx_buffer[TXBUFFERSIZE];				// Transmit buffer
uint8_t i2c_rx_buffer[RXBUFFERSIZE];				// Recieve buffer
_i2c_slv_rx i2c_slv_rx;                             // A union struct to hold the I2C slv RX
_i2c_slv_tx i2c_slv_tx;                             // A union struct to hold the I2C slv tx
_max6911_ctrl selector;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// Function prototypes
// -------------------------
// General Functionality
// -------------------------
void copy_array(volatile uint8_t *source, volatile uint8_t *dest, uint16_t count);
//HAL_StatusTypeDef i2c_write_cmd_data(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX);
bool i2c_slv_cmd_rx_tx_handle(void);
uint16_t max6911_read (I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t cmd_msb, uint8_t cmd_lsb);
bool ijc_detector_init(void);
bool cea_detector_init(void);


void ijc_dssd_ramp_loop(void);
void cea_dssd_ramp_loop(void);
// -------------------------
// GPIO and Board Enables
// -------------------------
void ht_enable_set(bool);
void cea_board_enable_set(bool);
bool cea_board_enable_get(void);
void ucd_board_enable_set(bool);
bool ucd_board_enable_get(void);
void ijc_board_enable_set(bool);
bool ijc_board_enable_get(void);

// I2C Slave Stuff
// -------------------------
//HAL_StatusTypeDef i2c_write_cmd_data(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX);
HAL_StatusTypeDef i2c_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef i2c_write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);
HAL_StatusTypeDef i2c_write_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef ucd_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);
HAL_StatusTypeDef ucd_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX);
//HAL_StatusTypeDef ucd_i2c_write_cmd_data(uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX);
HAL_StatusTypeDef ucd_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef ijc_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef ijc_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);
HAL_StatusTypeDef ijc_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX);


HAL_StatusTypeDef cea_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX);
HAL_StatusTypeDef cea_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX);
HAL_StatusTypeDef cea_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX);
// -------------------------
// I2C Master Stuff
// -------------------------
void i2c_slv_clear_buffer(uint8_t* buffer, uint8_t size);
void i2c_slv_init(void);
// -------------------------
// MAX6911 Stuff
// -------------------------
void max6911_set_ctrl1_register(_max6911_ctrl selector);
// -------------------------
// Deleteme
// -------------------------
uint8_t i = 0x00;
uint16_t max_value = 70;
uint8_t counter = 0;




//bool ijc_ramp_flag- = false;


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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  // Initialize the system
  i2c_slv_init(); // Initialize the I2C slave module
  bool ijc_init_status = ijc_detector_init();

  if(ijc_init_status == EXIT_FAILURE)
  {
	  while(1);
  }



  ht_enable_set(GPIO_PIN_SET);





  // Start the timer
  HAL_TIM_Base_Start_IT(&htim2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ijc_dssd_ramp_loop();
	  cea_dssd_ramp_loop();

	  if (Xfer_Complete ==1)                            // Check for the I2C read complete to have been executed
	  {

		  i2c_slv_cmd_rx_tx_handle();

		  HAL_Delay(1); 								// Delay for 1 ms
		  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) // Put I2C peripheral in listen mode process
		  {
			  Error_Handler();
		  }
		  Xfer_Complete =0;
	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 50;
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
  hi2c2.Init.Timing = 0x10909CEC;
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
  hi2c3.Init.Timing = 0x10909CEC;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TIMING_PIN_GPIO_Port, TIMING_PIN_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : TIMING_PIN_Pin */
  GPIO_InitStruct.Pin = TIMING_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TIMING_PIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//************************************
//            General
//************************************
void copy_array(volatile uint8_t *source, volatile uint8_t *dest, uint16_t count)
{
    uint16_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

// ###############################################
/*
HAL_StatusTypeDef i2c_write_cmd_data(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX)
{
	uint8_t out_ptr[10] = {0x00};
	out_ptr[0] = cmd;
	copy_array(data, &out_ptr[1], countTX);

	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, dev_addr, out_ptr, countTX + 1, I2C_TIMEOUT_DURATION);
	return(ret);
}
*/
// ###############################################

HAL_StatusTypeDef i2c_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX)
{
	// Read bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(hi2c, dev_addr, in_ptr, countRX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef i2c_write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef i2c_write_read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX)
{
	// The status used to indicate success/error
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t tx_attemps = I2C_TX_MAX_ATTEMPTS;
	uint8_t rx_attemps = I2C_RX_MAX_ATTEMPTS;

	do
	{
		// Write operation
		if(tx_attemps < I2C_TX_MAX_ATTEMPTS){HAL_Delay(I2C_TX_ATTEMPT_PERIOD);} // Delay if re-attempting I2C Operation
		status = i2c_write(hi2c, dev_addr, out_ptr, countTX);						// Perform I2C operation
		tx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (tx_attemps > 0));						    // Check the I2C operation status

	if(status == HAL_ERROR){return(status);}

	HAL_Delay(5);

	do
	{
		// Read operation
		status =  i2c_read(hi2c, dev_addr, in_ptr, countRX); 						// Delay if re-attempting I2C Operation
		if(rx_attemps < I2C_RX_MAX_ATTEMPTS){HAL_Delay(I2C_RX_ATTEMPT_PERIOD);}	// Perform I2C operation
		rx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (rx_attemps > 0));							// Check the I2C operation status

	return(status);
}

//************************************
//        Chips and Devs
//************************************

uint16_t max6911_read(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t cmd_msb, uint8_t cmd_lsb)
{
	uint16_t temp_data = 0;
	uint8_t rx_data[2] = {0x00, 0x00};
	uint8_t tx_data[2] = {0x00, 0x00};

	// Init the device ctrl reg1
	max6911_set_ctrl1_register(INIT_WITH_CURRENT_GAIN_8);

	// Write to the control register 1 and 2
	tx_data[0] = CONTROL_REGISTER_1;
	tx_data[1] = max6911.ctrl_reg_1.byte;
	i2c_write(hi2c, device_addr, &tx_data[0], 2);
	// -------------------------------------------
	tx_data[0] = CONTROL_REGISTER_2;
	tx_data[1] = max6911.ctrl_reg_2.byte;
	i2c_write(hi2c, device_addr, &tx_data[0], 2);

	// #############################################################################
	// The functions below do not work!
	// status = i2c_write_cmd_data
	// status = i2c_write_cmd_data(hi2c, device_addr, CONTROL_REGISTER_1, &max6911.ctrl_reg_1.byte, 1);
	// status = i2c_write_cmd_data(hi2c, device_addr, CONTROL_REGISTER_2, &max6911.ctrl_reg_2.byte, 1);
	// #############################################################################

	// Set up the device for normal fast read operation
	max6911_set_ctrl1_register(FASTREAD_NORMAL_OPERATION);

	// Write to the control register 1 and 2
	tx_data[0] = CONTROL_REGISTER_1;
	tx_data[1] = max6911.ctrl_reg_1.byte;
	i2c_write(hi2c, device_addr, &tx_data[0], 2);
	// -------------------------------------------
	tx_data[0] = CONTROL_REGISTER_2;
	tx_data[1] = max6911.ctrl_reg_2.byte;
	i2c_write(hi2c, device_addr, &tx_data[0], 2);

	// #############################################################################
	// Write to the control register 1 and 2
	// status = i2c_write_cmd_data(hi2c, device_addr, CONTROL_REGISTER_1, &max6911.ctrl_reg_1.byte, 1);
	// status = i2c_write_cmd_data(hi2c, device_addr, CONTROL_REGISTER_2, &max6911.ctrl_reg_2.byte, 1);
	// #############################################################################

	// Set up the read for MSB and LSB
	status = i2c_write_read(hi2c, device_addr, &cmd_msb, 1, &rx_data[0], 1);
	status = i2c_write_read(hi2c, device_addr, &cmd_lsb, 1, &rx_data[1], 1);

	temp_data = (rx_data[0]<<8) | (rx_data[1]);
	return(temp_data);

}


void max6911_set_ctrl1_register(_max6911_ctrl selector)
{

	switch(selector)
	{
		case(INIT_WITH_CURRENT_GAIN_8):
		{
			// Set up CTRL Reg 1 for gain = 8x measurements on
			max6911.ctrl_reg_1.bits.SHDN  = SHDN_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.LR    = LD_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.MODE  = AMP_COMP_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.MUX   = CHANNEL_A_GAIN_8x_DEC;
			max6911.ctrl_reg_2.bits.DTIM  = DTIM_1_MS_BITS_DEC;
			max6911.ctrl_reg_2.bits.RTIM  = RTIM_50_MS_BITS_DEC;
		}
		case(FASTREAD_NORMAL_OPERATION):
		{
		    // Configure the CTRL_REG general settings (Assumed general settings)
			max6911.ctrl_reg_1.bits.SHDN  = SHDN_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.LR    = LD_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.MODE  = AMP_COMP_NORMAL_OPERATION_DEC;
			max6911.ctrl_reg_1.bits.MUX   = CHANNEL_FAST_READ_DEC;
			max6911.ctrl_reg_2.bits.DTIM  = DTIM_1_MS_BITS_DEC;
			max6911.ctrl_reg_2.bits.RTIM  = RTIM_50_MS_BITS_DEC;
		}
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
		default:
			break;
	}
}


//************************************
//            HV
//************************************

// Board enable functions
void ht_enable_set(bool gpio_state)
{
	HAL_GPIO_WritePin(ENABLE_HT_GPIO_Port, ENABLE_HT_Pin, gpio_state);
}

void ijc_dssd_ramp_loop(void)
{
	// Control for the ramp up/down of the IJC voltage

	uint16_t max6911_measured_voltage = 0;
	uint8_t tx_data[2] = {0x00, 0x00};

	if(ijc_detector.ramp_flag == true && ijc_detector.hv_loop_enable == true)
	{
		// Read the MAX9611 voltage value
		max6911_measured_voltage = max6911_read(&hi2c2, ADDR_IJC_MAX9611_HV_VOLTAGE, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

		// Shift the measured voltage down by 4 bits
		max6911_measured_voltage = max6911_measured_voltage >> 4;

		// Get the current digipot value
		status = ijc_i2c_read(ADDR_IJC_DIGIPOT, &ijc_detector.hv_digipot_value, 1); 						// Delay if re-attempting I2C Operation

		// Check if the read value is greater than or less than the target value
		// This gets the direction of the ramp (up/down)
		if((ijc_detector.hv_targate_value > max6911_measured_voltage) && (max6911_measured_voltage < (ijc_detector.hv_targate_value - ijc_detector.hv_lower_deadband)))
		{
			// If the target is greater than the current value, increment the digipot value
			if (ijc_detector.hv_digipot_value <= 149)
			{
				ijc_detector.hv_digipot_value ++;

				// Write the digipot value - with 5 attempts
				tx_data[1] = ijc_detector.hv_digipot_value;
				HAL_StatusTypeDef status = ijc_i2c_write_read(ADDR_IJC_DIGIPOT, &tx_data[0], 2, &ijc_detector.hv_digipot_value, 1);
			}
		}
		else if ((ijc_detector.hv_targate_value < max6911_measured_voltage) && (max6911_measured_voltage > (ijc_detector.hv_targate_value + ijc_detector.hv_upper_deadband)))
		{
			if (ijc_detector.hv_digipot_value >= 1)
			{
				// If the target is less than the current value
				ijc_detector.hv_digipot_value --;

				// Write the digipot value - with 5 attempts
				tx_data[1] = ijc_detector.hv_digipot_value;

				HAL_StatusTypeDef status = ijc_i2c_write_read(ADDR_IJC_DIGIPOT, &tx_data[0], 2, &ijc_detector.hv_digipot_value, 1);
			}
		}
		else if((ijc_detector.hv_targate_value == 0) && (ijc_detector.hv_digipot_value > 0))
		{
			// If the value is 0 - continue ramping down to 0 on the digipot
			ijc_detector.hv_digipot_value --;

			// Write the digipot value - with 5 attempts
			tx_data[1] = ijc_detector.hv_digipot_value;

			HAL_StatusTypeDef status = ijc_i2c_write_read(ADDR_IJC_DIGIPOT, &tx_data[0], 2, &ijc_detector.hv_digipot_value, 1);
		}
		ijc_detector.ramp_flag  = false;
	}
}

void cea_dssd_ramp_loop(void)
{
	// Control for the ramp up/down of the CEA voltage

	uint16_t max6911_measured_voltage = 0;
	uint8_t tx_data[2] = {0x00, 0x00};

	if(cea_detector.ramp_flag == true)
	{
		// Read the MAX9611 voltage value
		max6911_measured_voltage = max6911_read(&hi2c2, ADDR_CEA_MAX9611_HV_VOLTAGE, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

		// Shift the measured voltage down by 4 bits
		max6911_measured_voltage = max6911_measured_voltage >> 4;

		// Get the current digipot value
		status = cea_i2c_read(ADDR_CEA_DIGIPOT, &cea_detector.hv_digipot_value, 1); 						// Delay if re-attempting I2C Operation

		// Check if the read value is greater than or less than the target value
		// This gets the direction of the ramp (up/down)
		if(cea_detector.hv_targate_value > max6911_measured_voltage)
		{
			// If the target is greater than the current value, increment the digipot value
			if (cea_detector.hv_digipot_value <= 149)
			{
				cea_detector.hv_digipot_value ++;

				// Write the digipot value - with 5 attempts
				tx_data[1] = cea_detector.hv_digipot_value;

				HAL_StatusTypeDef status = cea_i2c_write_read(ADDR_CEA_DIGIPOT, &tx_data[0], 2, &cea_detector.hv_digipot_value, 1);
			}
		}
		else if (cea_detector.hv_targate_value < max6911_measured_voltage)
		{
			if (cea_detector.hv_digipot_value >= 1)
			{
				// If the target is less than the current value
				cea_detector.hv_digipot_value --;

				// Write the digipot value - with 5 attempts
				tx_data[1] = cea_detector.hv_digipot_value;

				HAL_StatusTypeDef status = cea_i2c_write_read(ADDR_CEA_DIGIPOT, &tx_data[0], 2, &cea_detector.hv_digipot_value, 1);
			}
		}
		cea_detector.ramp_flag  = false;
	}
}


//************************************
//            UCD PSB
//************************************

// Board enable functions
void ucd_board_enable_set(bool gpio_state)
{
	HAL_GPIO_WritePin(ENABLE_5_UCD_GPIO_Port, ENABLE_5_UCD_Pin, gpio_state);
	ucd_detector.board_enable_state = gpio_state;

}

bool ucd_board_enable_get(void)
{
	ucd_detector.board_enable_state = HAL_GPIO_ReadPin(ENABLE_5_UCD_GPIO_Port, ENABLE_5_UCD_Pin);
	return(ucd_detector.board_enable_state);
}

HAL_StatusTypeDef ucd_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef ucd_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX)
{
	// Read bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c3, dev_addr, in_ptr, countRX, I2C_TIMEOUT_DURATION);
	return(ret);
}

// ###############################################
/*
HAL_StatusTypeDef ucd_i2c_write_cmd_data(uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX)
{

	uint8_t out_ptr[10] = {0x00};
	out_ptr[0] = cmd;
	copy_array(data, &out_ptr[1], countTX);

	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, out_ptr, countTX + 1, I2C_TIMEOUT_DURATION);
	return(ret);
}
*/
// ###############################################

HAL_StatusTypeDef ucd_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX)
{
	// The status used to indicate success/error
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t tx_attemps = I2C_TX_MAX_ATTEMPTS;
	uint8_t rx_attemps = I2C_RX_MAX_ATTEMPTS;

	do
	{
		// Write operation
		if(tx_attemps < I2C_TX_MAX_ATTEMPTS){HAL_Delay(I2C_TX_ATTEMPT_PERIOD);} // Delay if re-attempting I2C Operation
		status = ucd_i2c_write(dev_addr, out_ptr, countTX);						// Perform I2C operation
		tx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (tx_attemps > 0));						    // Check the I2C operation status

	if(status == HAL_ERROR){return(status);}

	HAL_Delay(5);

	do
	{
		// Read operation
		status =  ucd_i2c_read(dev_addr, in_ptr, countRX); 						// Delay if re-attempting I2C Operation
		if(rx_attemps < I2C_RX_MAX_ATTEMPTS){HAL_Delay(I2C_RX_ATTEMPT_PERIOD);}	// Perform I2C operation
		rx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (rx_attemps > 0));							// Check the I2C operation status

	return(status);
}


//************************************
//            IJC PSB
//************************************

// Init function
bool ijc_detector_init(void)
{

	// TODO - Add a status return - this is very important incase the init fails and the HV is enabled high!!!

	bool status = EXIT_SUCCESS;

	// Init the IJC lab detector
	ijc_detector.ramp_flag            = 0;
	ijc_detector.hv_max_digipot_value = IJC_MAX_DIGIPOT_VALUE - 1;
	ijc_detector.hv_min_digipot_value = IJC_MIN_DIGIPOT_VALUE - 1;
	ijc_detector.hv_lower_deadband 	  = IJC_LOWER_DEADBAND;
	ijc_detector.hv_upper_deadband 	  = IJC_UPPER_DEADBAND;
	ijc_detector.hv_digipot_value 	  = 0;
	ijc_detector.hv_targate_value 	  = 0;
	ijc_detector.board_enable_state   = 0;
	ijc_detector.hv_loop_enable 	  = 1;


	// Configure the board enable state
	ht_enable_set(GPIO_PIN_RESET);
	ijc_board_enable_set(GPIO_PIN_SET);

	HAL_Delay(100);

	// Reset the digipot value to 0
	uint8_t command[2] = {0x00, 0x00};
	HAL_StatusTypeDef digipot_set_stataus = ijc_i2c_write_read(ADDR_IJC_DIGIPOT, &command[0], 2, &ijc_detector.hv_digipot_value, 1);


	if (ijc_detector.hv_digipot_value != 0 || digipot_set_stataus == HAL_ERROR)
	{
		// Disable the loop enable flag
		ijc_detector.hv_loop_enable = false;
		status = EXIT_FAILURE;
		return(status);
	}
	else
	{
		status = EXIT_SUCCESS;
		return(status);
	}
}

// Board enable functions
void ijc_board_enable_set(bool gpio_state)
{
	HAL_GPIO_WritePin(ENABLE_2_IJC_GPIO_Port, ENABLE_2_IJC_Pin, gpio_state);
	ijc_detector.board_enable_state = gpio_state;
}

bool ijc_board_enable_get(void)
{
	ijc_detector.board_enable_state = HAL_GPIO_ReadPin(ENABLE_2_IJC_GPIO_Port, ENABLE_2_IJC_Pin);
	return(ijc_detector.board_enable_state);
}

HAL_StatusTypeDef ijc_i2c_write(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX)
{
	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, dev_addr, out_ptr, countTX, I2C_TIMEOUT_DURATION);
	return(ret);
}

HAL_StatusTypeDef ijc_i2c_read(uint8_t dev_addr, uint8_t *in_ptr, uint16_t countRX)
{
	// Read bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c2, dev_addr, in_ptr, countRX, I2C_TIMEOUT_DURATION);
	return(ret);
}

// ###############################################
/*
HAL_StatusTypeDef ijc_i2c_write_cmd_data(uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t countTX)
{
	uint8_t out_ptr[10] = {0x00};
	out_ptr[0] = cmd;
	copy_array(data, &out_ptr[1], countTX);

	// Write bytes over I2C
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, dev_addr, out_ptr, countTX + 1, I2C_TIMEOUT_DURATION);
	return(ret);
}
*/
// ###############################################

HAL_StatusTypeDef ijc_i2c_write_read(uint8_t dev_addr, uint8_t *out_ptr, uint16_t countTX, uint8_t *in_ptr, uint16_t countRX)
{
	// The status used to indicate success/error
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t tx_attemps = I2C_TX_MAX_ATTEMPTS;
	uint8_t rx_attemps = I2C_RX_MAX_ATTEMPTS;

	do
	{
		// Write operation
		if(tx_attemps < I2C_TX_MAX_ATTEMPTS){HAL_Delay(I2C_TX_ATTEMPT_PERIOD);} // Delay if re-attempting I2C Operation
		status = ijc_i2c_write(dev_addr, out_ptr, countTX);						// Perform I2C operation
		tx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (tx_attemps > 0));						    // Check the I2C operation status

	if(status == HAL_ERROR){return(status);}

	HAL_Delay(5);

	do
	{
		// Read operation
		status =  ijc_i2c_read(dev_addr, in_ptr, countRX); 						// Delay if re-attempting I2C Operation
		if(rx_attemps < I2C_RX_MAX_ATTEMPTS){HAL_Delay(I2C_RX_ATTEMPT_PERIOD);}	// Perform I2C operation
		rx_attemps --;															// Decrement the attempt counter
	}while((status == HAL_ERROR) && (rx_attemps > 0));							// Check the I2C operation status

	return(status);
}


//************************************
//            CEA PSB
//************************************

// Board enable functions
void cea_board_enable_set(bool gpio_state)
{
	HAL_GPIO_WritePin(ENABLE_3_CEA_GPIO_Port, ENABLE_3_CEA_Pin, gpio_state);
	cea_detector.board_enable_state = gpio_state;
}

bool cea_board_enable_get(void)
{
	cea_detector.board_enable_state = HAL_GPIO_ReadPin(ENABLE_3_CEA_GPIO_Port, ENABLE_3_CEA_Pin);
	return(cea_detector.board_enable_state);
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

//bool place_ijc_dssd_hv_into_safe_state(void)
//{

	// If the board enable pin is reset
		// Disable the HV enable pin.
		// Set the board enable pin
		// Set the digipot value to 0
		// Disable the board enable pin
		// return(0)

	// If the board enable pin is set
		// If HV enable pin is set
			// Get the digipot value
				// if the digipot value is > 0
					// Ramp down to 0 from that value
					// Disable the HV enable pin.
					// Disable the enable pin
					// return(0)

		// If HV enable pin is reset
			// Set the digipot value to 0
			// Disable the enable pin
			// return(0)
//}


//************************************
//            I2C Slave
//************************************

bool i2c_slv_cmd_rx_tx_handle(void)
{
	bool status = EXIT_SUCCESS;


	// SOME FUNCTION HERE TO HANDLE RX/TX
    switch(i2c_slv_rx.bytes.cmd)
    {
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
		//                     UCD Detector Commands
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
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c3, ADDR_UCD_MAX9611_AVDD, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_UCD_AVDD_CURRENT):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c3, ADDR_UCD_MAX9611_AVDD, CSA_DATA_BYTE_MSB, CSA_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	//                     CEA Detector Commands
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	case(CMD_CEA_ENABLE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				cea_detector.board_enable_state = (uint16_t)cea_board_enable_get(); // Read the state enable pin
				i2c_slv_tx.data = cea_detector.board_enable_state; 					// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				// Write the state of the enable pin
				if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_SET)
				{
					cea_board_enable_set(GPIO_PIN_SET);								// Set the state of the pin
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_RESET)
				{
					cea_board_enable_set(GPIO_PIN_RESET);                           // Set the state of the pin
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
    	case(CMD_CEA_HV_LOOP_ENABLE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				i2c_slv_tx.data = cea_detector.hv_loop_enable; 						// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				// Write the state of the enable pin
				if(i2c_slv_rx.bytes.data_byte_lsb == 1)
				{
					cea_detector.hv_loop_enable = true;								// Set the state of the flag
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else if(i2c_slv_rx.bytes.data_byte_lsb == 0)
				{
					cea_detector.hv_loop_enable = false;						    // Set the state of the flag
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
    	case(CMD_CEA_HV_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_CEA_MAX9611_HV_VOLTAGE, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_CEA_HV_CURRENT):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_CEA_MAX9611_HV_CURRENT, CSA_DATA_BYTE_MSB, CSA_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_CEA_TARGET_HV_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				i2c_slv_tx.data = cea_detector.hv_targate_value;        // Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				if(i2c_slv_rx.bytes.data_byte_msb <= 0x0F)
				{
					// Read the data from the buffer
					cea_detector.hv_targate_value = (i2c_slv_rx.bytes.data_byte_msb << 8) |
													 i2c_slv_rx.bytes.data_byte_lsb;
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else
				{
					i2c_slv_tx.data = CMD_FAIL_OP_RESP;
					status =  EXIT_FAILURE;
					return(status);
				}
			}
			break;
		}
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	//                     IJC Detector Commands
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	case(CMD_IJC_ENABLE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				ijc_detector.board_enable_state = (uint16_t)ijc_board_enable_get(); // Read the state enable pin
				i2c_slv_tx.data = ijc_detector.board_enable_state; 					// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				// Write the state of the enable pin
				if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_SET)
				{
					ijc_board_enable_set(GPIO_PIN_SET);								// Set the state of the pin
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else if(i2c_slv_rx.bytes.data_byte_lsb == GPIO_PIN_RESET)
				{
					ijc_board_enable_set(GPIO_PIN_RESET);                           // Set the state of the pin
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
    	case(CMD_IJC_HV_LOOP_ENABLE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				i2c_slv_tx.data = ijc_detector.hv_loop_enable; 						// Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				// Write the state of the enable pin
				if(i2c_slv_rx.bytes.data_byte_lsb == 1)
				{
					ijc_detector.hv_loop_enable = true;								// Set the state of the flag
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else if(i2c_slv_rx.bytes.data_byte_lsb == 0)
				{
					ijc_detector.hv_loop_enable = false;						    // Set the state of the flag
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
    	case(CMD_IJC_1_5_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_1_5, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_IJC_1_5_CURRENT):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_1_5, CSA_DATA_BYTE_MSB, CSA_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				i2c_slv_tx.data = CMD_FAIL_OP_RESP;
				status =  EXIT_FAILURE;
				{
				    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
				}
				return(status);
			}
			break;
		}
		// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	case(CMD_IJC_2_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_2, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_IJC_2_CURRENT):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_2, CSA_DATA_BYTE_MSB, CSA_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_IJC_HV_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_HV_VOLTAGE, RSP_DATA_BYTE_MSB, RSP_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_IJC_HV_CURRENT):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				// read the UCD max6911 AVDD device
				uint16_t dataread = 0;
				dataread = max6911_read(&hi2c2, ADDR_IJC_MAX9611_HV_CURRENT, CSA_DATA_BYTE_MSB, CSA_DATA_BYTE_LSB);

				// Load the MSB and LSB into the TX register buffer
				i2c_slv_tx.data = dataread;  		                    // Prepare the date into the transmit

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
    	case(CMD_IJC_TARGET_HV_VOLTAGE):
		{
			if(i2c_slv_rx.bytes.rw_state == CMD_READ)
			{
				i2c_slv_tx.data = ijc_detector.hv_targate_value;        // Prepare the date into the transmit
				return(status);
			}
			else if (i2c_slv_rx.bytes.rw_state == CMD_WRITE)
			{
				if(i2c_slv_rx.bytes.data_byte_msb <= 0x0F)
				{
					// Read the data from the buffer
					ijc_detector.hv_targate_value = (i2c_slv_rx.bytes.data_byte_msb << 8) |
													 i2c_slv_rx.bytes.data_byte_lsb;
					i2c_slv_tx.data = CMD_SUCCESS_RESP;
					return(status);
				}
				else
				{
					i2c_slv_tx.data = CMD_FAIL_OP_RESP;
					status =  EXIT_FAILURE;
					return(status);
				}
			}
			break;
		}
    	// ---------------------------------------------------------------------
		// ---------------------------------------------------------------------
    	default:
			status =  EXIT_FAILURE;
			return(status);
	}

    // If no command is matched - return failure
	status =  EXIT_FAILURE;
	return(status);
}



void i2c_slv_init(void)
{
	// Clear the Tx and Rx buffers before use
	i2c_slv_clear_buffer(&i2c_tx_buffer[0], TXBUFFERSIZE);
	i2c_slv_clear_buffer(&i2c_rx_buffer[0], RXBUFFERSIZE);

	if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
	{
		/* Transfer error in reception process */
		Error_Handler();
	}
}


void i2c_slv_clear_buffer(uint8_t* buffer, uint8_t size)
{

	uint8_t idx = 0; // An index

	for(idx = 0; idx < size; idx++)
	{
		// Cycle through and clear the buffer array
		*buffer = 0x00;
		buffer++;
	}

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
	Xfer_Complete = 1;        // Set flag for RX completion
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
	// Handle the I2C receive callback complete
	i2c_slv_rx.data = (i2c_rx_buffer[0] << 24) 	 // Load the data from the RX buffer
					| (i2c_rx_buffer[1] << 16)
					| (i2c_rx_buffer[2] << 8)
					| (i2c_rx_buffer[3]);

	// Load the init/fail value into the tx buffer
	i2c_slv_tx.data = CMD_FAIL_OP_INPROG_RESP;

	Xfer_Complete = 1;                           // Set flag for RX completion
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
	"i2c_tx_buffer" buffer */

	// Update the TX buffer
	i2c_tx_buffer[0] = i2c_slv_tx.bytes.byte_1;
	i2c_tx_buffer[1] = i2c_slv_tx.bytes.byte_2;
	//copy_array(&i2c_slv_tx.bytes.byte_1, i2c_tx_buffer[0], TXBUFFERSIZE);

	if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)i2c_tx_buffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}

  }
  else
  {

	/*##- Put I2C peripheral in reception process ###########################*/
	if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)i2c_rx_buffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
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
	//HAL_I2C_EnableListen_IT(&hi2c1);
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	counter ++;

	// Set the flags responsable for running through the DSSD loop
	ijc_detector.ramp_flag = true;
	cea_detector.ramp_flag = true;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
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
