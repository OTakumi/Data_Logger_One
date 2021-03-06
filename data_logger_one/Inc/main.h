/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Startup_Message(void);
void Uart_Message(char*);
void Get_Temp_Humid(float*, uint16_t*);
void Led_Bring(int16_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define XL345_CS_Pin GPIO_PIN_0
#define XL345_CS_GPIO_Port GPIOA
#define XL345_INT1_Pin GPIO_PIN_1
#define XL345_INT1_GPIO_Port GPIOA
#define XL345_INT2_Pin GPIO_PIN_2
#define XL345_INT2_GPIO_Port GPIOA
#define TestPad3_Pin GPIO_PIN_3
#define TestPad3_GPIO_Port GPIOA
#define XL372_CS_Pin GPIO_PIN_0
#define XL372_CS_GPIO_Port GPIOB
#define XL372_INT1_Pin GPIO_PIN_1
#define XL372_INT1_GPIO_Port GPIOB
#define XL372_INT2_Pin GPIO_PIN_2
#define XL372_INT2_GPIO_Port GPIOB
#define TestPad7_Pin GPIO_PIN_10
#define TestPad7_GPIO_Port GPIOB
#define MX25_CS_Pin GPIO_PIN_11
#define MX25_CS_GPIO_Port GPIOB
#define TestPad4_Pin GPIO_PIN_15
#define TestPad4_GPIO_Port GPIOA
#define Mode_LED_Pin GPIO_PIN_3
#define Mode_LED_GPIO_Port GPIOB
#define Mode_SW_Pin GPIO_PIN_4
#define Mode_SW_GPIO_Port GPIOB
#define TestPad5_Pin GPIO_PIN_8
#define TestPad5_GPIO_Port GPIOB
#define TestPad6_Pin GPIO_PIN_9
#define TestPad6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
