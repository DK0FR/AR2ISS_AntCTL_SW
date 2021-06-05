/*
 * usartClass.cpp
 *
 *  Created on: 30.05.2021
 *      Author: Soeren
 */

#include "usartClass.hpp"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os2.h"

void uartName(UART_HandleTypeDef *handle, uint8_t* string, uint8_t size);



//usartClass::usartClass(TaskParams rxTask, TaskParams txTask, UART_HandleTypeDef *huart){
//	}



usartClass::usartClass(){



//	this->m_hRxQueue
//	this-m_hTxQueue
//	this-RxTask()
//	this-TxTask()

//	xTaskCreate(this->TxTask, "Task", 2048, this, 5, NULL);

}


void usartClass::initTasks(TaskParams rxTask, TaskParams txTask, UART_HandleTypeDef *huart){


	m_huart = huart;
	uint8_t nameBuffer[20];
	uint8_t tempBuffer[20];
	uartName(m_huart,tempBuffer,20);


	m_hTxQueue = xQueueCreate(txTask.queueLength,txTask.queueElementSize);
	m_hRxQueue = xQueueCreate(rxTask.queueLength,rxTask.queueElementSize);


	snprintf((char*)nameBuffer,20,"%sTxQueue",(char*)tempBuffer);
	vQueueAddToRegistry(m_hTxQueue, (const char*) nameBuffer);
	snprintf((char*)nameBuffer,20,"%sRxQueue",(char*)tempBuffer);
	vQueueAddToRegistry(m_hRxQueue, (const char*) nameBuffer);

	snprintf((char*)nameBuffer,20,"%sTxTask",(char*)tempBuffer);
	xTaskCreate(this->TxTask, (const char*)nameBuffer, 2048, this, 5, NULL);
	snprintf((char*)nameBuffer,20,"%sRxTask",(char*)tempBuffer);
	xTaskCreate(this->RxTask, (const char*)nameBuffer, 2048, this, 5, NULL);

}

usartClass::~usartClass(){

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
	HAL_GPIO_TogglePin(ROT_LEFT_GPIO_Port, ROT_LEFT_Pin);
	osDelay(200);

}
void usartClass::RxTask(){
  // start task
	HAL_GPIO_TogglePin(ROT_UP_GPIO_Port, ROT_DOWN_Pin);
	osDelay(200);
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


