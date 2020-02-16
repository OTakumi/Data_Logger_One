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
 * �ｿｽE �ｿｽW�ｿｽ�ｿｽ�ｿｽC�ｿｽ�ｿｽ�ｿｽZ�ｿｽ�ｿｽ�ｿｽT�ｿｽ[�ｿｽﾆ会ｿｽ�ｿｽ�ｿｽ�ｿｽx�ｿｽZ�ｿｽ�ｿｽ�ｿｽT�ｿｽ[�ｿｽ�ｿｽ�ｿｽ�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽ�ｿｽ謫ｾ�ｿｽ�ｿｽ�ｿｽ�ｿｽ
 * �ｿｽE �ｿｽ謫ｾ�ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽcsv�ｿｽ`�ｿｽ�ｿｽ�ｿｽ�ｿｽFlashMemory�ｿｽﾉ保托ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽ
 * �ｿｽE FlashMemory�ｿｽﾉ保托ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽUART�ｿｽ�ｿｽPC�ｿｽﾉ托ｿｽ�ｿｽM�ｿｽ�ｿｽ�ｿｽ�ｿｽ
 * �ｿｽE �ｿｽ�ｿｽ�ｿｽ闥��ｿｽﾉゑｿｽUART�ｿｽﾅ托ｿｽ�ｿｽM�ｿｽA�ｿｽm�ｿｽF�ｿｽ\�ｿｽ�ｿｽ�ｿｽﾅゑｿｽ�ｿｽ�ｿｽ
 * �ｿｽE �ｿｽW�ｿｽ�ｿｽ�ｿｽC�ｿｽ�ｿｽ�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽ0.1ms�ｿｽ�ｿｽ�ｿｽﾆに取得�ｿｽ�ｿｽ�ｿｽ�ｿｽ
 * �ｿｽE �ｿｽ�ｿｽ�ｿｽx�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽ1min�ｿｽ�ｿｽ�ｿｽﾆに取得�ｿｽ�ｿｽ�ｿｽ�ｿｽ
 * �ｿｽE SW�ｿｽﾅ�ｿｽ�ｿｽ[�ｿｽh�ｿｽﾘ替ゑｿｽ�ｿｽ�ｿｽ
 * 	�ｿｽE �ｿｽ�ｿｽ�ｿｽ[�ｿｽh1�ｿｽFFlashMemory�ｿｽﾉ保托ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽ�ｿｽf�ｿｽ[�ｿｽ^�ｿｽ�ｿｽPC�ｿｽﾉ托ｿｽ�ｿｽM�ｿｽ�ｿｽ�ｿｽ驛ゑｿｽ[�ｿｽh
 * 	�ｿｽE �ｿｽ�ｿｽ�ｿｽ[�ｿｽh2�ｿｽF�ｿｽﾊ常測�ｿｽ�ｿｽ�ｿｽ�ｿｽs�ｿｽ�ｿｽ
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
void Read_Sw_Status(int8_t*);
void ADXL345_init(uint8_t*);
void XL345_readXYZ(int16_t*);
void ADXL345_SPI_Read(uint8_t*, uint8_t);
void ADXL345_SPI_Write(uint8_t, uint8_t);
void ADXL372_init(uint8_t*);
void adxl372_settings(void);
void XL372_readXYZ(int16_t*);
void ADXL372_SPI_Read(uint8_t*, uint8_t);
void ADXL372_SPI_Write(uint8_t, uint8_t);
void Uart_Message(char*);
void Get_Temp_Humid(float*, uint16_t*);
void Led_Bring(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char MESSAGE[0xff] = { };
uint8_t xl345_spi_error_flg = 0xff;
uint8_t xl372_spi_error_flg = 0xff;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	int8_t sw_flag = 0;
	uint16_t humid = 0;
	float temp = 0.0;
	int16_t xl345_xyz_data[3] = { };
	int16_t xl372_xyz_data[3] = { };
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
	// Mode SW
	// SW = ON
	// SW = OFF
	Read_Sw_Status(&sw_flag);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	// Mode SW
	if (sw_flag == 1)
	{

	}

	// Mode SW�ｿｽ�ｿｽOFF(SW_Flag = 0)
	if (sw_flag == 0)
	{
		// Startup_Message();
		while (xl345_spi_error_flg != 0 || xl372_spi_error_flg != 0)
		{
			ADXL345_init(&xl345_spi_error_flg);
			HAL_Delay(10);
			ADXL372_init(&xl372_spi_error_flg);
		}
		while (1)
		{
			Get_Temp_Humid(&temp, &humid);// Get the temperature data and the humidity data

			for (uint8_t i = 0; i < 9; i++)
			{
				XL345_readXYZ(xl345_xyz_data);// Get the data stored in ADXL345 FIFO.
				XL372_readXYZ(xl372_xyz_data);// Get the data stored in ADXL372 FIFO.

				sprintf(MESSAGE, "%6d, %6d, %6d, ", xl345_xyz_data[0],
						xl345_xyz_data[1], xl345_xyz_data[2]);
				Uart_Message(MESSAGE);
				sprintf(MESSAGE, "%6d, %6d, %6d \r\n", xl372_xyz_data[0],
						xl372_xyz_data[1], xl372_xyz_data[2]);
				Uart_Message(MESSAGE);
				HAL_Delay(500);
			}
			sprintf(MESSAGE, "%6d, %6d, %6d, ", xl345_xyz_data[0],
					xl345_xyz_data[1], xl345_xyz_data[2]);
			Uart_Message(MESSAGE);
			sprintf(MESSAGE, "%6d, %6d, %6d,  ", xl372_xyz_data[0],
					xl372_xyz_data[1], xl372_xyz_data[2]);
			Uart_Message(MESSAGE);
			sprintf(MESSAGE, "%4.2f, %4d \r\n", temp, humid);
			Uart_Message(MESSAGE);
			Led_Bring();
			HAL_Delay(500);
			// FlashMemory
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
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
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
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

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 10000 - 1;
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
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, XL345_CS_Pin | TestPad3_Pin | TestPad4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	XL372_CS_Pin | TestPad7_Pin | Mode_LED_Pin | TestPad5_Pin | TestPad6_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : XL345_CS_Pin TestPad3_Pin TestPad4_Pin */
	GPIO_InitStruct.Pin = XL345_CS_Pin | TestPad3_Pin | TestPad4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : XL345_INT1_Pin XL345_INT2_Pin */
	GPIO_InitStruct.Pin = XL345_INT1_Pin | XL345_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : XL372_CS_Pin TestPad7_Pin TestPad5_Pin TestPad6_Pin */
	GPIO_InitStruct.Pin = XL372_CS_Pin | TestPad7_Pin | TestPad5_Pin
			| TestPad6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : XL372_INT1_Pin XL372_INT2_Pin Mode_SW_Pin */
	GPIO_InitStruct.Pin = XL372_INT1_Pin | XL372_INT2_Pin | Mode_SW_Pin;
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

void Read_Sw_Status(int8_t* sw_status)
{

}

/*--------------------------------------
 * ADXL345
 * -------------------------------------*/
void ADXL345_init(uint8_t *xl345_spi_error_flg)
{
	/*
	 *  ADXL345 setting
	 */

	ADXL345_SPI_Write(XL345_DATA_FORMAT & 0x7f, 0X0b);		//BIT6: SPI4 line mode (default); BIT5: interrupt level 0/1 (high/low active); BIT0-1: range=16g
	ADXL345_SPI_Write(XL345_POWER_CTL & 0x7f, 0x08);		//BIT3=0/1: (measurement mode/standby mode); BIT2=0/1: (work/hibernate);
	ADXL345_SPI_Write(XL345_BW_RATE & 0x7f, 0x0e);			//low 4 bits: output data rate=1600 (at this rate, SPI rate should be set >=2M); BIT4=0/1 (low power/normal)
	ADXL345_SPI_Write(XL345_INT_ENABLE & 0x7f, 0x00);		//Interrupt function setting: not enabled
	ADXL345_SPI_Write(XL345_INT_MAP & 0x7f, 0x00); 			//Set the interrupt mapping to the INT1 pin or the INT2 pin.
	ADXL345_SPI_Write(XL345_FIFO_CTL & 0x7f, 0x80);

	ADXL345_SPI_Write(XL345_OFSX & 0x7f, 0x00); 		//XYZ offset adjustment
	ADXL345_SPI_Write(XL345_OFSY & 0x7f, 0x00);
	ADXL345_SPI_Write(XL345_OFSZ & 0x7f, 0x00);

	// Get device id
	uint8_t xl345_read_data[3] = { };
	xl345_read_data[0] = XL345_DEVID | 0xc0;
	xl345_read_data[1] = 0x00;

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, xl345_read_data, sizeof(xl345_read_data), TIME_OUT);
	HAL_Delay(5);
	XL345_CS_HIGH();

	uint8_t device_id = xl345_read_data[1];

	if (device_id != XL345_I_M_DEVID)
	{
		*xl345_spi_error_flg = 1;
	}
	else
	{
		*xl345_spi_error_flg = 0;
	}
}

/*---------- Get ADXL345 Acceleration Data ---------- */
void XL345_readXYZ(int16_t *xl345_data_buf)
{
	// Setting the data format
	ADXL345_SPI_Write(XL345_DATA_FORMAT & 0x7f, 0x0f);

	// Read multibit
	uint8_t xl345_accel_data[6] = { };
	xl345_accel_data[0] = XL345_DATAX0 | 0xc0;

	/* data read multi bits XL345_DATAX0 ~ XL345_DATAZ1 */
	ADXL345_SPI_Read(xl345_accel_data, sizeof(xl345_accel_data));

	xl345_data_buf[0] = ((uint16_t) xl345_accel_data[1] << 8)
			+ xl345_accel_data[0];
	xl345_data_buf[1] = ((uint16_t) xl345_accel_data[3] << 8)
			+ xl345_accel_data[2];
	xl345_data_buf[2] = ((uint16_t) xl345_accel_data[5] << 8)
			+ xl345_accel_data[4];
}

void ADXL345_SPI_Read(uint8_t *read_data_buf, uint8_t buf_size)
{
	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, read_data_buf, buf_size, TIME_OUT);
	HAL_Delay(5);
	XL345_CS_HIGH();
}

void ADXL345_SPI_Write(uint8_t addr, uint8_t data)
{
	uint8_t xl345_write_data_buf[2] = { };
	xl345_write_data_buf[0] = addr;
	xl345_write_data_buf[1] = data;

	XL345_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, xl345_write_data_buf, sizeof(xl345_write_data_buf), TIME_OUT);
	HAL_Delay(5);
	XL345_CS_HIGH();
}

/*--------------------------------------
 * ADXL372
 * -------------------------------------*/
void ADXL372_init(uint8_t *xl372_spi_error_flg)
{
	/*
	 * ADXL372
	 */
	uint8_t xl372_rx_data_buf[3] = { };

	// setting the function
	adxl372_settings();

	// get power mode status
	xl372_rx_data_buf[0] = ADXL372_POWER_CTL << 1 | 0x01;
	ADXL372_SPI_Read(xl372_rx_data_buf, sizeof(xl372_rx_data_buf));

	// get device id
	xl372_rx_data_buf[0] = ADXL372_PARTID << 1 | 0x01;
	ADXL372_SPI_Read(xl372_rx_data_buf, sizeof(xl372_rx_data_buf));

	if (xl372_rx_data_buf[1] != ADXL372_PARTID_VAL)
	{
		// Uart_Message("ADXL372 SPI Error\r\n");
		*xl372_spi_error_flg = 1;
	}
	else
	{
		// Uart_Message("ADXL372 SPI OK\r\n");
		*xl372_spi_error_flg = 0;
	}
}

void adxl372_settings(void)
{
	ADXL372_SPI_Write(ADXL372_POWER_CTL << 1 & 0xfe,
			ADXL372_POWER_CTL_MODE(ADXL372_STANDBY));			// STANDBY mode
	ADXL372_SPI_Write(ADXL372_RESET << 1 & 0xfe, ADXL372_RESET_CODE);		// Device Reset

	// FIFO Setting
	// FIFO_CTL_FORMAT_MODE = XYZ_FIFO
	// FIFO_CTL_MODE_MODE = FIFO_STREAMED
	ADXL372_SPI_Write(ADXL372_FIFO_CTL << 1 & 0xfe,
			ADXL372_FIFO_CTL_FORMAT_MODE(ADXL372_XYZ_FIFO)
					| ADXL372_FIFO_CTL_MODE_MODE(ADXL372_FIFO_STREAMED) | 0);

	// Measurement CTL Setting
	// AUTOSLEEP_MODE(6bit) = Disable(0)
	// LINKLOOP_MODE(5:4bit) = default mode(0)
	// LOW_NOISE_MODE(3bit) = Enable(1)
	// BANDWIDTH_MODE(2:0bit) = 3200Hz(100)
	ADXL372_SPI_Write(ADXL372_MEASURE << 1 & 0xfe,
			ADXL372_MEASURE_AUTOSLEEP_MODE(0)
			| ADXL372_MEASURE_LINKLOOP_MODE(0)
			| ADXL372_MEASURE_LOW_NOISE_MODE(1)
			| ADXL372_MEASURE_BANDWIDTH_MODE(ADXL372_BW_3200HZ));

	// Timing Setting
	// Output data rate(7:5bit) = 3200Hz ODR(011)
	// WAKE_UP_RATE_MODE(4:2bit) = 2048ms(100)
	// EXT_CLK_MODE(1bit) = Disable(0)
	// EXT_SYNC_MODE(0bit) = Disable(0)
	ADXL372_SPI_Write(ADXL372_TIMING << 1 & 0xfe,
			ADXL372_TIMING_ODR_MODE(ADXL372_ODR_3200HZ)
			| ADXL372_TIMING_WAKE_UP_RATE_MODE(ADXL372_WUR_2048ms)
			| ADXL372_TIMING_EXT_CLK_MODE(0)
			| ADXL372_TIMING_EXT_SYNC_MODE(0));

	// Power CTL Setting
	// 7, 6bit = Reserved
	// INSTANT_ON_TH_MODE(5bit) = low instant on threshold(0)
	// FIL_SETTLE_MODE(4bit) = 370ms(0)
	// POWER_CTL_MODE(1:0bit) = FULL_BW_MEASUREMENT(11)
	ADXL372_SPI_Write(ADXL372_POWER_CTL << 1 & 0xfe,
			ADXL372_POWER_CTL_INSTANT_ON_TH_MODE(ADXL372_INSTANT_ON_LOW_TH)
			| ADXL372_POWER_CTL_FIL_SETTLE_MODE(ADXL372_FILTER_SETTLE_370)
			| ADXL372_POWER_CTL_LPF_DIS_MODE(1)
			| ADXL372_POWER_CTL_HPF_DIS_MODE(0)
			| ADXL372_POWER_CTL_MODE(ADXL372_FULL_BW_MEASUREMENT));	// FULL_BW_MEASUREMENT mode
}

/*---------- Get ADXL372 Acceleration Data ---------- */
void XL372_readXYZ(int16_t *xl372_data_buf)
{
	uint8_t xl372_buf[7] = { };
	uint8_t xl372_xyz_data[24] = { };

	xl372_buf[0] = ADXL372_FIFO_ENTRIES_2 << 1 | 0x01;
	ADXL372_SPI_Read(xl372_buf, sizeof(xl372_buf));
	xl372_buf[0] = ADXL372_FIFO_ENTRIES_1 << 1 | 0x01;
	ADXL372_SPI_Read(xl372_buf, sizeof(xl372_buf));
	xl372_buf[0] = ADXL372_FIFO_SAMPLES << 1 | 0x01;
	ADXL372_SPI_Read(xl372_buf, sizeof(xl372_buf));

	xl372_xyz_data[0] = ADXL372_FIFO_DATA << 1 | 0x01;
	ADXL372_SPI_Read(xl372_xyz_data, sizeof(xl372_xyz_data));

	for (uint8_t cnt = 0; cnt < 5; cnt += 6)
	{
		xl372_buf[0] = ADXL372_X_DATA_H << 1 | 0x01;
		ADXL372_SPI_Read(xl372_buf, sizeof(xl372_buf));

		xl372_data_buf[0] = ((int16_t) xl372_buf[cnt] << 4)
				| (xl372_buf[cnt + 1] >> 4);
		xl372_data_buf[1] = ((int16_t) xl372_buf[cnt + 2] << 4)
				| (xl372_buf[cnt + 3] >> 4);
		xl372_data_buf[2] = ((int16_t) xl372_buf[cnt + 4] << 4)
				| (xl372_buf[cnt + 5] >> 4);
	}
}

void ADXL372_SPI_Read(uint8_t *read_data_buf, uint8_t buf_size)
{
	ADXL372_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi1, read_data_buf, buf_size, TIME_OUT);
	HAL_Delay(5);
	ADXL372_CS_HIGH();
}

void ADXL372_SPI_Write(uint8_t addr, uint8_t data)
{
	uint8_t xl372_write_data_buf[3] = { };
	xl372_write_data_buf[0] = addr;
	xl372_write_data_buf[1] = data;

	ADXL372_CS_LOW();
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1, xl372_write_data_buf, sizeof(xl372_write_data_buf), TIME_OUT);
	HAL_Delay(5);
	ADXL372_CS_HIGH();
}

/*---------- Get Temperature and Humidity ---------- */
void Get_Temp_Humid(float *temp, uint16_t *humid)
{
	uint8_t i2c_tx_buf[8] = { };
	uint8_t Temp_data_buf[4] = { };
	uint8_t Humid_data_buf[4] = { };

	i2c_tx_buf[0] = Temperature_Not_Hold;// Measure Temperature, No Hold Master Mode
	i2c_tx_buf[1] = Humidity_Not_Hold;// Measure Relative Humidity, No Hold Master Mode
	uint16_t device_addr = Si7006_ADDERSS << 1;

	// Get the Temperature data
	HAL_I2C_Master_Transmit(&hi2c1, device_addr, &i2c_tx_buf[0],
			sizeof(Temp_data_buf), TIME_OUT);
	HAL_I2C_Master_Receive(&hi2c1, device_addr, Temp_data_buf,
			sizeof(Temp_data_buf), TIME_OUT);
	*temp = (Temp_data_buf[0] << 8) + Temp_data_buf[1];
	*temp = ((175.72 * (*temp)) / 65536) - 46.85;

	// Get the Humidity data
	HAL_I2C_Master_Transmit(&hi2c1, device_addr, &i2c_tx_buf[1],
			sizeof(Humid_data_buf), TIME_OUT);
	HAL_I2C_Master_Receive(&hi2c1, device_addr, Humid_data_buf,
			sizeof(Humid_data_buf), TIME_OUT);
	*humid = (Humid_data_buf[0] << 8) + Humid_data_buf[1];
	*humid = ((125 * (*humid)) / 65536) - 6;
}

/*---------- LED Bring ---------- */
void Led_Bring(void)
{
	HAL_GPIO_WritePin(Mode_LED_GPIO_Port, Mode_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Mode_LED_GPIO_Port, Mode_LED_Pin, GPIO_PIN_RESET);
}

/*---------- UART message ---------- */
void Uart_Message(char *message)
{
	/*
	 * Send messages with UART
	 */
	char tx_message[0xff] = { 0 };
	sprintf(tx_message, "%s", message);
	HAL_UART_Transmit(&huart1, (uint8_t*) tx_message, sizeof(tx_message),
	TIME_OUT);
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
