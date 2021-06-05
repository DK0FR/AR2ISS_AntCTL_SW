/*
 * uartHandlers.cpp
 *
 *  Created on: 20.03.2019
 *      Author: talruso
 */



#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "uartHandlers.h"
#include "limits.h"
#include<stdlib.h>
#include "spi.h"
#include "FreeRTOS.h"
#include "queue.h"



extern QueueHandle_t xQueueUART3RX;
extern QueueHandle_t xQueueUART3TX;

uint8_t uartRXBuff[UARTRXBUFFERSIZE];

extern osThreadId uartRxTaskHandle;
extern osThreadId uartTxTaskHandle;

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;



uint8_t sendData[UARTRXBUFFERSIZE] = {0};


double stepAngles[10] = {0};
uint16_t stepTimesMs[10] = {0};





//enum StepTypes { Unkown, SingleStep, MultiStep, SingleDuty, Dutysweep};


StepTypes StepType = Unkown;
uint8_t StepVCNr = 0;

/* USER CODE BEGIN Header_uartRXHandler */
/**
* @brief Function implementing the uartRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartRXHandler */
void uartRXHandler(void const * argument)
{
	/* USER CODE BEGIN uartRXHandler */
	//	uint8_t uartRXBuff[UARTBUFFSIZE];
	uint8_t localRXBuff[UARTRXBUFFERSIZE];



	/* Infinite loop */
	for(;;)
	{
		if (xQueueReceive(xQueueUART3RX, localRXBuff, portMAX_DELAY) == pdTRUE) { // If no new message has arrived within 500ms, check if receive is still running

				memset(localRXBuff,0,10);
			}
		}
		/* USER CODE END uartRXHandler */
	}

/* USER CODE BEGIN Header_uartTXHandler */
/**
* @brief Function implementing the uartTX thread.
* @param argument: Not used
* @retval None
*/

void uartTXHandler(void const * argument)
{
	uint8_t abc = 0xAF;
	UartTXStruct localTXData;
	localTXData.bufPtr = &abc;
//	TaskHandle_t txTask =
  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(xQueueUART3TX, (uint8_t*)&(localTXData), portMAX_DELAY)) {	 //wait for data
	  			if (localTXData.bufPtr[localTXData.size - 1] != '\n') {										 //add NL and CR if not already in string
	  				localTXData.bufPtr[localTXData.size] = '\r';
	  				localTXData.bufPtr[localTXData.size + 1] = '\n';
	  				localTXData.size += 2;
	  			}
	  			xTaskNotifyStateClear((TaskHandle_t)uartTxTaskHandle);
	  			HAL_UART_Transmit_DMA(&huart3, (uint8_t*) localTXData.bufPtr, localTXData.size);			 //send data via DMA to save CPU power

	  			ulTaskNotifyTake( pdTRUE, 500 / portTICK_PERIOD_MS);                        //block UART till Data is send
	  			//notification send from stm32f4xx_it.c  HAL_UART_TxCpltCallback(
	  			if (huart3.gState != HAL_UART_STATE_READY)
	  				huart3.gState = HAL_UART_STATE_READY;							 //fix HAL bug, that blocks the UART indefinitely.

	  			if(localTXData.usedInRTOS){
	  				vPortFree(localTXData.bufPtr); //delete[] localTXData.bufPtr when memory was allocated from task;
	  			}else{
	  				free(localTXData.bufPtr); //delete[] localTXData.bufPtr when memory was allocated by interrupt;
	  			}
	  		  }

  }
  /* USER CODE END uartTXHandler */
}







