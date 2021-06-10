/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "app_freertos.h"
#include "usartClass.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

usartClass RS485Uart;

extern UART_HandleTypeDef huart1;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256,
  .priority = (osPriority_t) osPriorityNormal
};
/* Definitions for pttControlTask */
osThreadId_t pttControlTaskHandle;
const osThreadAttr_t pttControlTask_attributes = {
  .name = "pttControlTask",
  .stack_size = 256,
  .priority = (osPriority_t) osPriorityRealtime
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void  rxCallback( UART_HandleTypeDef * huart, uint8_t* data, uint16_t length);
/* USER CODE END FunctionPrototypes */
void StartDefaultTask(void *argument);
void pttControl(void *argument);


/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of pttControlTask */
  pttControlTaskHandle = osThreadNew(pttControl, NULL, &pttControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */


  TaskParams paramRx = {
	  .stackSize = 256,
	  .prio = 10,
	  .queueLength = 10,
	  .queueElementLength = 50
  };
  TaskParams paramTx = {
	  .stackSize = 256,
	  .prio = 10,
	  .queueLength = 10,
	  .queueElementLength = 0
  };
	LedsSetup leds = {
		.RXPORT = RJ45_LED2_GPIO_Port,
		.TXPORT = Rj45_LED1_GPIO_Port,
		.RXPIN  =  RJ45_LED2_Pin,
		.TXPIN  =  Rj45_LED1_Pin,
	};


  RS485Uart.initTasks(paramRx, paramTx, &huart1,&rxCallback);
  RS485Uart.setupLeds(leds);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	  HAL_GPIO_TogglePin(AMP_GPIO_Port, AMP_Pin);
	  osDelay(500);
//	  HAL_GPIO_TogglePin(RJ45_LED2_GPIO_Port, RJ45_LED2_Pin);
	  osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_pttControl */
/**
* @brief Function implementing the pttControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pttControl */
__weak void pttControl(void *argument)
{
  /* USER CODE BEGIN pttControl */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(500);
  }
  /* USER CODE END pttControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void  rxCallback( UART_HandleTypeDef * huart, uint8_t* data, uint16_t length){
	HAL_GPIO_TogglePin(Rj45_LED1_GPIO_Port, Rj45_LED1_Pin);

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
