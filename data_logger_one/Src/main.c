/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * @System Detail
 * ・ ジャイロセンサーと温湿度センサーからデータを取得する
 * ・ 取得したデータをcsv形式でFlashMemoryに保存する
 * ・ FlashMemoryに保存したデータをUARTでPCに送信する
 * ・ 測定中にもUARTで送信、確認表示できる
 * ・ ジャイロデータは0.1msごとに取得する
 * ・ 温度データは1minごとに取得する
 * ・ SWでモード切替する
 * 	・ モード1：FlashMemoryに保存したデータをPCに送信するモード
 * 	・ モード2：通常測定を行う
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32l0xx.h"
#include "adxl345.h"
#include "adxl372.h"
#include "si7006_a20.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIME_OUT		0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Startup_Message(void);
int8_t ADXL345_init(void);
int8_t ADXL372_init(void);
void XL345_readXYZ(int16_t*, int16_t*, int16_t*);
uint8_t ADXL345_read_byte(uint8_t);
void Uart_Message(char*);
void Get_Temp_Humid(int*);
void Led_Bring(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char MESSAGE[0xff] = {};
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	Startup_Message();

	while(ADXL345_init() != 0 && ADXL372_init() != 0)
	{
		ADXL345_init();
		ADXL372_init();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		int16_t x = 0, y = 0, z = 0;
		int *si7006_data_buf[4] = { };

		Get_Temp_Humid(*si7006_data_buf);
		Led_Bring();
		XL345_readXYZ(&x, &y, &z);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x200009FE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  Uart_Message("I2C has error");
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 8;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XL345_CS_Pin|TestPad3_Pin|TestPad4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XL372_CS_Pin|TestPad7_Pin|Mode_LED_Pin|TestPad5_Pin 
                          |TestPad6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : XL345_CS_Pin TestPad3_Pin TestPad4_Pin */
  GPIO_InitStruct.Pin = XL345_CS_Pin|TestPad3_Pin|TestPad4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : XL345_INT1_Pin XL345_INT2_Pin */
  GPIO_InitStruct.Pin = XL345_INT1_Pin|XL345_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : XL372_CS_Pin TestPad7_Pin TestPad5_Pin TestPad6_Pin */
  GPIO_InitStruct.Pin = XL372_CS_Pin|TestPad7_Pin|TestPad5_Pin|TestPad6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : XL372_INT1_Pin XL372_INT2_Pin Mode_SW_Pin */
  GPIO_InitStruct.Pin = XL372_INT1_Pin|XL372_INT2_Pin|Mode_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Mode_LED_Pin */
  GPIO_InitStruct.Pin = Mode_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mode_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*---------- Startup message ---------- */
void Startup_Message(void)
{
	char message[] = "Start Data logging\r\n";
	Uart_Message(message);
}

/*---------- ADXL345 Init ---------- */
int8_t ADXL345_init(void)
{
	/*
	 * SPI通信確認
	 */

	uint8_t xl345_tx_buf[3] = { XL345_DEVID, XL345_DATA_FORMAT, 0xff };
	uint8_t format_data = 0xAC;
	uint8_t xl345_rx_buf[8] = { };
	char message[0x20] = { };

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, &xl345_tx_buf[1], 0x08, TIME_OUT);
	HAL_SPI_Transmit(&hspi1, &format_data, 0x08, TIME_OUT);
	XL345_CS_HIGH();
	HAL_Delay(5);

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_TransmitReceive(&hspi1, &xl345_tx_buf[0], (uint8_t *)xl345_rx_buf, 0x08, TIME_OUT);
	HAL_Delay(5);
	XL345_CS_HIGH();

	uint8_t xl345_rx_buf_data = xl345_rx_buf[5] << 1;
	xl345_rx_buf_data = xl345_rx_buf_data + 1;

	sprintf(message, "rx data = %x \r\n", xl345_rx_buf_data);
	Uart_Message(message);

	if(xl345_rx_buf_data != XL345_I_M_DEVID)
	{
		Uart_Message("ADXL345 SPI Error\r\n");
		return 1;
	}
	else
	{
		Uart_Message("ADXL345 SPI OK\r\n");
		return 0;
	}
}

/*---------- ADXL372 Init ---------- */
int8_t ADXL372_init(void)
{
	/*
	 * SPI通信確認
	 */
	uint8_t device_id = XL372_DEVID_MST;
	uint8_t xl372_rx_buf[6] = {  };
	uint16_t data_size = 0x0f;

	XL372_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_TransmitReceive(&hspi1, &device_id, (uint8_t *)xl372_rx_buf, data_size, TIME_OUT);
	HAL_Delay(5);
	XL372_CS_HIGH();

	if(xl372_rx_buf[3] != XL372_I_M_DEVID)
	{
		Uart_Message("ADXL372 SPI Error\r\n");
		return 1;
	}
	else
	{
		Uart_Message("ADXL372 SPI OK\r\n");
		return 0;
	}
}
/*---------- Get ADXL345 Acceleration Data ---------- */
void XL345_readXYZ(int16_t *xl345_x, int16_t *xl345_y, int16_t *xl345_z)
{
	uint8_t xl345_buf[6];

	xl345_buf[0] = ADXL345_read_byte(0x32);
	xl345_buf[1] = ADXL345_read_byte(0x33);

	xl345_buf[2] = ADXL345_read_byte(0x34);
	xl345_buf[3] = ADXL345_read_byte(0x35);

	xl345_buf[4] = ADXL345_read_byte(0x36);
	xl345_buf[5] = ADXL345_read_byte(0x37);

	*xl345_x = ((uint16_t)xl345_buf[1] << 8) + xl345_buf[0];
	*xl345_y = ((uint16_t)xl345_buf[3] << 8) + xl345_buf[2];
	*xl345_z = ((uint16_t)xl345_buf[5] << 8) + xl345_buf[4];
}

uint8_t ADXL345_read_byte(uint8_t address)
{
	uint8_t xl345_tx_address = address;
	uint8_t xl345_rx_buf[8] = { };

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_TransmitReceive(&hspi1, &xl345_tx_address, (uint8_t *)xl345_rx_buf, 0x08, TIME_OUT);
	HAL_Delay(5);
	XL345_CS_HIGH();

	return 0;
}

/*---------- Get Temperature and Humidity ---------- */
void Get_Temp_Humid(int *si7006_data)
{
	/*
	 * si7006から温度、湿度データを取得する
	 */
	uint8_t i2c_tx_buf[8] = { 0x00 };
	uint8_t Humid_data_buf[8] = { 0x00 };
	i2c_tx_buf[0] = Humidity_Not_Hold;
	uint8_t Temp_data_buf[8] = { 0x00 };
	i2c_tx_buf[1] = Temperature_Not_Hold;
	uint16_t device_addr = Si7006_ADDERSS << 1;

	HAL_I2C_Master_Transmit(&hi2c1, device_addr, &i2c_tx_buf[0], 0x08, TIME_OUT);
	HAL_I2C_Master_Receive(&hi2c1, device_addr, Humid_data_buf, 0x08, TIME_OUT);

	HAL_I2C_Master_Transmit(&hi2c1, device_addr, &i2c_tx_buf[1], 0x08, TIME_OUT);
	HAL_I2C_Master_Receive(&hi2c1, device_addr, Temp_data_buf, 0x08, TIME_OUT);

	sprintf(MESSAGE, "Temp = %x ", Temp_data_buf[0]);
	Uart_Message(MESSAGE);
	sprintf(MESSAGE, "Humid = %x\r\n", Humid_data_buf[0]);
	Uart_Message(MESSAGE);
}

/*---------- LED Bring ---------- */
void Led_Bring(void)
{
	/*
	 * LED点滅設定
	 */

	HAL_GPIO_WritePin(Mode_LED_GPIO_Port, Mode_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Mode_LED_GPIO_Port, Mode_LED_Pin, GPIO_PIN_RESET);
}
/*---------- UART message ---------- */
void Uart_Message(char *message)
{
	/*
	 * UARTでメッセージを送信
	 * 引数：message
	 * 		表示したい文字列
	 */
	char tx_message[0xff] = { 0 };
	sprintf(tx_message, "%s", message);
	HAL_UART_Transmit(&huart1, (uint8_t*)tx_message, sizeof(tx_message), TIME_OUT);
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
	Uart_Message("Have Some Error");

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
