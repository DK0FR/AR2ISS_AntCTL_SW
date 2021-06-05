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
#include "uartHandlers.h"
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
QueueHandle_t xQueueUART3RX;
QueueHandle_t xQueueUART3TX;



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for uartRxTask */
osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = {
  .name = "uartRxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256
};
/* Definitions for uartTxTask */
osThreadId_t uartTxTaskHandle;
const osThreadAttr_t uartTxTask_attributes = {
  .name = "uartTxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256
};
/* Definitions for rotorControlTas */
osThreadId_t rotorControlTasHandle;
const osThreadAttr_t rotorControlTas_attributes = {
  .name = "rotorControlTas",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128
};
/* Definitions for pttControlTask */
osThreadId_t pttControlTaskHandle;
const osThreadAttr_t pttControlTask_attributes = {
  .name = "pttControlTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void uartRx(void *argument);
void uartTx(void *argument);
void rotorControl(void *argument);
void pttControl(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
	xQueueUART3RX = xQueueCreate(3,UARTRXBUFFERSIZE*sizeof(uint8_t));
	xQueueUART3TX = xQueueCreate(10,sizeof(UartTXStruct));

	vQueueAddToRegistry(xQueueUART3RX, (const char*) "Q_UART3RX");
	vQueueAddToRegistry(xQueueUART3TX, (const char*) "Q_UART3TX");
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uartRxTask */
  uartRxTaskHandle = osThreadNew(uartRx, NULL, &uartRxTask_attributes);

  /* creation of uartTxTask */
  uartTxTaskHandle = osThreadNew(uartTx, NULL, &uartTxTask_attributes);

  /* creation of rotorControlTas */
  rotorControlTasHandle = osThreadNew(rotorControl, NULL, &rotorControlTas_attributes);

  /* creation of pttControlTask */
  pttControlTaskHandle = osThreadNew(pttControl, NULL, &pttControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_uartRx */
/**
* @brief Function implementing the uartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartRx */
__weak void uartRx(void *argument)
{
  /* USER CODE BEGIN uartRx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uartRx */
}

/* USER CODE BEGIN Header_uartTx */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartTx */
__weak void uartTx(void *argument)
{
  /* USER CODE BEGIN uartTx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uartTx */
}

/* USER CODE BEGIN Header_rotorControl */
/**
* @brief Function implementing the rotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rotorControl */
__weak void rotorControl(void *argument)
{
  /* USER CODE BEGIN rotorControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rotorControl */
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
    osDelay(1);
  }
  /* USER CODE END pttControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
