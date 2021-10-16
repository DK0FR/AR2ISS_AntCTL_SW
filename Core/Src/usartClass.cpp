/*
 * usartClass.cpp
 *
 *  Created on: 30.05.2021
 *      Author: Soeren
 */

#include "usartClass.hpp"
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include "main.h"
#include "cmsis_os2.h"
#include "stm32l5xx_hal.h"


void uartName(UART_HandleTypeDef *handle, uint8_t* string, uint8_t size);


usartClass::usartClass():m_hTxTask(NULL),m_hRxTask(NULL),m_hTxQueue(NULL),m_hRxQueue(NULL){

	m_huart = NULL;
//	m_hRxQueue = NULL;
//	m_hTxQueue = NULL;
//	m_hRxTask = NULL;
//	m_hRxTask = NULL;


}


usartClass::~usartClass(){

}

void usartClass::initTasks(TaskParams rxTask, TaskParams txTask, UART_HandleTypeDef *huart, void (*rxCallback)(UART_HandleTypeDef*, uint8_t*, uint16_t)){


	m_huart = huart;
	uint8_t nameBuffer[20];
	uint8_t tempBuffer[8];
	uartName(m_huart,tempBuffer,20);
	m_LedsOn = false;

	m_TxTaskCallback = rxCallback;

	m_txTaskParam = txTask;
	m_rxTaskParam = rxTask;

	/* create queues. either with dynamic memory or static memory*/

	if(m_txTaskParam.queueElementLength == 0){
		m_hTxQueue = xQueueCreate(txTask.queueLength,sizeof(UartHeapStruct));
	} else {
		m_hTxQueue = xQueueCreate(txTask.queueLength,txTask.queueElementLength*sizeof(uint8_t));
	}
	m_hRxQueue = xQueueCreate(rxTask.queueLength,rxTask.queueElementLength*sizeof(uint8_t));



	snprintf((char*)nameBuffer,20,"%sTxQueue",(char*)tempBuffer);
	vQueueAddToRegistry(m_hTxQueue, (const char*) nameBuffer);
	snprintf((char*)nameBuffer,20,"%sRxQueue",(char*)tempBuffer);
	vQueueAddToRegistry(m_hRxQueue, (const char*) nameBuffer);

	snprintf((char*)nameBuffer,20,"%sTxTask",(char*)tempBuffer);
	xTaskCreate(this->TxTask, (const char*)nameBuffer, 2048, this, 5, NULL);
	snprintf((char*)nameBuffer,20,"%sRxTask",(char*)tempBuffer);
	xTaskCreate(this->RxTask, (const char*)nameBuffer, 2048, this, 5, NULL);

}



void usartClass::setupLeds(LedsSetup leds){
	m_LedsOn	= true;
	m_Leds 		= leds;
}

void usartClass::TxTask(void *argument){
  // start task
		static_cast<usartClass*>(argument)->TxTask();
}

void usartClass::RxTask(void *argument){
  // start task
  static_cast<usartClass*>(argument)->RxTask();
}

void usartClass::TxTask(){
  // start task
	if(m_txTaskParam.queueElementLength == 0)
		TxTaskDynamic();
	else
		TxTaskStatic();
}

void usartClass::TxTaskDynamic(){
	uint8_t abc = 0xAF;
	UartHeapStruct localTXData;
	localTXData.bufPtr = &abc;
	/* Infinite loop */
	for(;;)
	{
		if (xQueueReceive(m_hTxQueue, (uint8_t*)&(localTXData), 750 /portTICK_PERIOD_MS)) {	 //wait for data
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.TXPORT, m_Leds.TXPIN, GPIO_PIN_SET);

			if (localTXData.bufPtr[localTXData.size - 1] != '\n') {										 //add NL and CR if not already in string
				localTXData.bufPtr[localTXData.size] = '\r';
				localTXData.bufPtr[localTXData.size + 1] = '\n';
				localTXData.size += 2;
			}
			xTaskNotifyStateClear((TaskHandle_t)m_hTxTask);
			HAL_UART_Transmit_DMA(m_huart, (uint8_t*) localTXData.bufPtr, localTXData.size);			 //send data via DMA to save CPU power

			ulTaskNotifyTake( pdTRUE, 500 / portTICK_PERIOD_MS);                        //block UART till Data is send
			//notification send from stm32f4xx_it.c  HAL_UART_TxCpltCallback(
			if (m_huart->gState != HAL_UART_STATE_READY)
				m_huart->gState = HAL_UART_STATE_READY;							 //fix HAL bug, that blocks the UART indefinitely.

			if(localTXData.usedInRTOS){
				vPortFree(localTXData.bufPtr); //delete[] localTXData.bufPtr when memory was allocated from task;
			}else{
				free(localTXData.bufPtr); //delete[] localTXData.bufPtr when memory was allocated by interrupt;
			}
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.TXPORT, m_Leds.TXPIN, GPIO_PIN_RESET);
		}else {
//			if(m_LedsOn)
//				HAL_GPIO_TogglePin(m_Leds.TXPORT, m_Leds.TXPIN);
		}

	}
}

void usartClass::TxTaskStatic(){

	uint8_t localTXBuff[m_txTaskParam.queueElementLength];
	/* Infinite loop */
	for(;;)
	{
		if (xQueueReceive(m_hTxQueue, localTXBuff, 750 /portTICK_PERIOD_MS)) {	 //wait for data
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.TXPORT, m_Leds.TXPIN, GPIO_PIN_SET);
			uint16_t length = strlen((const char*)localTXBuff);
			if (localTXBuff[length - 1] != '\n') {										 //add NL and CR if not already in string
				localTXBuff[length++] = '\r';
				localTXBuff[length++] = '\n';
			}
			xTaskNotifyStateClear((TaskHandle_t)m_hTxTask);
			HAL_UART_Transmit_DMA(m_huart, (uint8_t*) localTXBuff, length);		//send data via DMA to save CPU power

			ulTaskNotifyTake( pdTRUE, 500 / portTICK_PERIOD_MS);                // block UART till Data is send
			//notification send from stm32f4xx_it.c  HAL_UART_TxCpltCallback(
			if (m_huart->gState != HAL_UART_STATE_READY)
				m_huart->gState = HAL_UART_STATE_READY;							// fix HAL bug, that blocks the UART indefinitely.
			memset(localTXBuff,0,m_txTaskParam.queueElementLength);				// reset buffer
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.TXPORT, m_Leds.TXPIN, GPIO_PIN_RESET);
		} else {
//			if(m_LedsOn)
//				HAL_GPIO_TogglePin(m_Leds.TXPORT, m_Leds.TXPIN);
		}
	}
}




void usartClass::RxTask(){
	uint8_t localRXBuff[m_rxTaskParam.queueElementLength];

	m_rxDMABuffer = (uint8_t*)pvPortMalloc(m_rxTaskParam.queueElementLength*sizeof(uint8_t));
	HAL_UART_Receive_DMA(m_huart,(unsigned char*) m_rxDMABuffer,m_rxTaskParam.queueElementLength); // start receiving

	/* Infinite loop */
	for(;;)
	{
		if (xQueueReceive(m_hRxQueue, localRXBuff, 500 /portTICK_PERIOD_MS ) == pdTRUE) { // If no new message has arrived within 500ms, check if receive is still running
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.RXPORT, m_Leds.RXPIN, GPIO_PIN_SET);
			m_TxTaskCallback(m_huart, localRXBuff, m_rxTaskParam.queueElementLength);
			memset(localRXBuff,0,10);
			if(m_LedsOn)
				HAL_GPIO_WritePin(m_Leds.TXPORT, m_Leds.TXPIN, GPIO_PIN_RESET);
		} else {
			if(m_LedsOn)
				HAL_GPIO_TogglePin(m_Leds.RXPORT, m_Leds.RXPIN);
		}
	}
	vPortFree(m_rxDMABuffer);
}


void usartClass::rxCpltCallback(){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xQueueGenericSendFromISR(m_hRxQueue,m_rxDMABuffer,&xHigherPriorityTaskWoken,queueSEND_TO_BACK);

	memset(m_rxDMABuffer,0,m_rxTaskParam.queueElementLength);


	HAL_UART_Receive_DMA(m_huart,(unsigned char*) m_rxDMABuffer,m_rxTaskParam.queueElementLength); // start receiving
	if( xHigherPriorityTaskWoken ){
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void usartClass::txCpltCallback(){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	vTaskNotifyGiveFromISR(m_hTxTask, &xHigherPriorityTaskWoken);

	if( xHigherPriorityTaskWoken ){
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}



QueueHandle_t usartClass::getRxQueueHandle(){
	return m_hRxQueue;
}
QueueHandle_t usartClass::getTxQueueHandle(){
	return m_hTxQueue;
}
TaskHandle_t usartClass::getTxTaskHandle(){
	return m_hRxTask;
}
TaskHandle_t usartClass::getRxTaskHandle(){
	return m_hTxTask;
}
UART_HandleTypeDef* usartClass::getRxUartHandle(){
	return m_huart;
}







void uartName(UART_HandleTypeDef *handle, uint8_t* string, uint8_t size){

	if(handle->Instance == USART1)
		snprintf((char*)string,size,"USART1");
	else if(handle->Instance == USART2)
		snprintf((char*)string,size,"USART2");
	else if(handle->Instance == USART3)
		snprintf((char*)string,size,"USART3");
	else if(handle->Instance == UART4)
		snprintf((char*)string,size,"UART4");
	else if(handle->Instance == UART5)
		snprintf((char*)string,size,"UART5");

}


