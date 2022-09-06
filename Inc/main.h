/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
extern TIM_HandleTypeDef htim2;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_LED_Pin GPIO_PIN_13
#define BLUE_LED_GPIO_Port GPIOC
#define RIGHT6_Pin GPIO_PIN_4
#define RIGHT6_GPIO_Port GPIOA
#define RIGHT5_Pin GPIO_PIN_5
#define RIGHT5_GPIO_Port GPIOA
#define RIGHT4_Pin GPIO_PIN_6
#define RIGHT4_GPIO_Port GPIOA
#define RIGHT3_Pin GPIO_PIN_7
#define RIGHT3_GPIO_Port GPIOA
#define RIGHT2_Pin GPIO_PIN_0
#define RIGHT2_GPIO_Port GPIOB
#define RIGHT1_Pin GPIO_PIN_1
#define RIGHT1_GPIO_Port GPIOB
#define HAT1_Pin GPIO_PIN_12
#define HAT1_GPIO_Port GPIOB
#define HAT2_Pin GPIO_PIN_13
#define HAT2_GPIO_Port GPIOB
#define HAT3_Pin GPIO_PIN_14
#define HAT3_GPIO_Port GPIOB
#define HAT4_Pin GPIO_PIN_15
#define HAT4_GPIO_Port GPIOB
#define HAT5_Pin GPIO_PIN_8
#define HAT5_GPIO_Port GPIOA
#define LEFT1_Pin GPIO_PIN_15
#define LEFT1_GPIO_Port GPIOA
#define LEFT2_Pin GPIO_PIN_3
#define LEFT2_GPIO_Port GPIOB
#define LEFT3_Pin GPIO_PIN_4
#define LEFT3_GPIO_Port GPIOB
#define LEFT4_Pin GPIO_PIN_5
#define LEFT4_GPIO_Port GPIOB
#define LEFT5_Pin GPIO_PIN_6
#define LEFT5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
