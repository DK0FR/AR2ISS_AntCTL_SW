/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l5xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Rj45_LED1_Pin GPIO_PIN_13
#define Rj45_LED1_GPIO_Port GPIOC
#define RJ45_LED2_Pin GPIO_PIN_14
#define RJ45_LED2_GPIO_Port GPIOC
#define PreAmp_Pin GPIO_PIN_15
#define PreAmp_GPIO_Port GPIOC
#define PTT_Main_Pin GPIO_PIN_6
#define PTT_Main_GPIO_Port GPIOA
#define PTT_Sub_Pin GPIO_PIN_7
#define PTT_Sub_GPIO_Port GPIOA
#define LDC_EN_Pin GPIO_PIN_0
#define LDC_EN_GPIO_Port GPIOB
#define LCD_SEL_Pin GPIO_PIN_1
#define LCD_SEL_GPIO_Port GPIOB
#define ROT_UP_Pin GPIO_PIN_11
#define ROT_UP_GPIO_Port GPIOB
#define ROT_DOWN_Pin GPIO_PIN_12
#define ROT_DOWN_GPIO_Port GPIOB
#define ROT_LEFT_Pin GPIO_PIN_13
#define ROT_LEFT_GPIO_Port GPIOB
#define ROT_RIGHT_Pin GPIO_PIN_14
#define ROT_RIGHT_GPIO_Port GPIOB
#define Relay_SwitchB_Pin GPIO_PIN_11
#define Relay_SwitchB_GPIO_Port GPIOA
#define Relay_SwitchA_Pin GPIO_PIN_12
#define Relay_SwitchA_GPIO_Port GPIOA
#define Coax_NC_Pin GPIO_PIN_4
#define Coax_NC_GPIO_Port GPIOB
#define Coax_NO_Pin GPIO_PIN_5
#define Coax_NO_GPIO_Port GPIOB
#define AMP_Pin GPIO_PIN_7
#define AMP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
