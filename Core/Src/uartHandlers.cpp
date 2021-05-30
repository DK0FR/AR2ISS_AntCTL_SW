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
#include "DRV8835_H-Bridge.h"
#include "uartHandlers.h"
#include "ArduinoJson.h"
#include "PID_DervOfInput.h"
#include "limits.h"
#include<stdlib.h>
#include "spi.h"
#include "LiquidLens.h"


extern PID VC1_PID;
extern PID VC2_PID;


extern QueueHandle_t xQueueUART3RX;
extern QueueHandle_t xQueueUART3TX;
extern osThreadId uartTXTaskHandle;
extern osThreadId commandInterpreHandle;

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;

extern DRV8835 DRV_X_Axis;
extern DRV8835 DRV_Y_Axis;

uint8_t sendData[UARTRXBUFFERSIZE] = {0};


double stepAngles[10] = {0};
uint16_t stepTimesMs[10] = {0};

const int capacity = JSON_OBJECT_SIZE(10) + 10 * JSON_OBJECT_SIZE(10);
const int capacity2 = JSON_OBJECT_SIZE(10);

DynamicJsonDocument JSONcommand(capacity);
//JsonDocument JSONStepObj;



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

			DeserializationError err = deserializeJson(JSONcommand, localRXBuff);
			if (err) {
				osDelay(1);
			}


			if(JSONcommand.containsKey("Ping")){
				UartTXStruct localTXData = {.bufPtr =(uint8_t*) pvPortMalloc(UARTTXBUFFERSIZE * sizeof(uint8_t)), .usedInRTOS = true , .size = 0};
				localTXData.size = sprintf((char*)localTXData.bufPtr,"{\"Ping\":\"Pong\"}");
				if(xQueueSendToBack(xQueueUART3TX,&localTXData,0) == errQUEUE_FULL){
					vPortFree(localTXData.bufPtr);
				}
			}

			if(JSONcommand.containsKey("SetPoint")){ // decode new Set value with no data logging
				JsonObject JSONSetPoint = JSONcommand["SetPoint"].as<JsonObject>();

				uint8_t VCNR = JSONSetPoint["VC_Aktor"].as<uint8_t>();

				if(VCNR == 1){
					setValue1 = JSONSetPoint["Angle"].as<double>();
				}else if(VCNR == 2){
					setValue2 = JSONSetPoint["Angle"].as<double>();
				}

			}

			if(JSONcommand.containsKey("SimpleStepObj")){ // decode the Step function Object

				JsonObject JSONSimpleStepObj = JSONcommand["SimpleStepObj"].as<JsonObject>();
				StepType = (StepTypes)JSONSimpleStepObj["Type"].as<uint8_t>();
				xTaskNotifyGive(commandInterpreHandle);
			}

			if(JSONcommand.containsKey("StepObj")){ // decode the Step function Object

				JsonObject JSONStepObj = JSONcommand["StepObj"].as<JsonObject>();

				StepType = (StepTypes)JSONStepObj["Type"].as<uint8_t>();
				StepVCNr = JSONStepObj["VC_Aktor"].as<uint8_t>();

				stepAngles[0] = JSONStepObj["StepTo"].as<double>(); // if it isn't an array, get the value anyway
				for (uint8_t i = 0; i< JSONStepObj["StepTo"].size(); i++){
					stepAngles[i]=JSONStepObj["StepTo"][i].as<double>();
				}
				stepTimesMs[0] = JSONStepObj["dtMs"].as<double>(); // if it isn't an array, get the value anyway
				for (uint8_t i = 0; i< JSONStepObj["dtMs"].size(); i++){
					stepTimesMs[i]=JSONStepObj["dtMs"][i].as<uint16_t>();
				}

				recordTimeBeforeMS = JSONStepObj["PreRecMs"].as<uint16_t>();
				recordTimeAfterMS = JSONStepObj["PostRecMs"].as<uint16_t>();

				xTaskNotifyGive(commandInterpreHandle);

			}


			if(JSONcommand.containsKey("PIDValues")){ // decode the PID change command
				JsonObject JSONPIDObj = JSONcommand["PIDValues"].as<JsonObject>();
				double Ptemp = JSONPIDObj["P"].as<double>();
				double Itemp = JSONPIDObj["I"].as<double>();
				double Dtemp = JSONPIDObj["D"].as<double>();

				if (JSONPIDObj["SelPID"].as<uint8_t>() == 1){
					VC1_PID.SetTunings(Ptemp,Itemp,Dtemp);
				} else if(JSONPIDObj["SelPID"].as<uint8_t>() == 2){
					VC2_PID.SetTunings(Ptemp,Itemp,Dtemp);
				}
			}

			int8_t enablePIDS = JSONcommand["enablePIDs"].as<int8_t>();
			if(enablePIDS == 1){
				DRV_X_Axis.enablePWM();
				DRV_Y_Axis.enablePWM();
				HAL_TIM_Base_Start_IT(&htim14);

			}
			else if(enablePIDS == -1){
				DRV_X_Axis.disablePWM();
				DRV_Y_Axis.disablePWM();
				HAL_TIM_Base_Stop_IT(&htim14);
			}



			if(JSONcommand.containsKey("setLLVolt")){ // decode the PID change command
				uint32_t LLVolt = JSONcommand["setLLVolt"].as<uint32_t>();
				cmd_lensdriver_overwriteVoltage(LLVolt);

			}
			uint8_t setSensorZero = JSONcommand["setSensorZero"].as<uint8_t>();
			if(setSensorZero > 0){
				setSensorZeroPoint(setSensorZero);
			}

			uint8_t enableJoystick = JSONcommand["enableJoystick"].as<uint8_t>();
			if(enableJoystick == 1 ){
		  	HAL_ADC_Start_IT(&hadc1);
			}
			if(enableJoystick == -1 ){
		  	HAL_ADC_Stop_IT(&hadc1);
			}
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
	uint16_t length = 0;
	uint8_t abc = 0xAF;
	UartTXStruct localTXData;
	localTXData.bufPtr = &abc;
  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(xQueueUART3TX, (uint8_t*)&(localTXData), portMAX_DELAY)) {	 //wait for data
	  			if (localTXData.bufPtr[localTXData.size - 1] != '\n') {										 //add NL and CR if not already in string
	  				localTXData.bufPtr[localTXData.size] = '\r';
	  				localTXData.bufPtr[localTXData.size + 1] = '\n';
	  				localTXData.size += 2;
	  			}
	  			xTaskNotifyStateClear(uartTXTaskHandle);
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


/* USER CODE BEGIN Header_startCommandInterpreter */
/**
* @brief Function implementing the commandInterpre thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startCommandInterpreter */
void startCommandInterpreter(void const * argument)
{
  /* USER CODE BEGIN startCommandInterpreter */

  	cmd_lensdriver_power_on();
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );                        //block until Command is received


	  switch (StepType) {
		case SingleStep:
			HAL_TIM_Base_Start_IT(&htim13); // start 10kHz clock
			HAL_TIM_Base_Start_IT(&htim6); // start data-logging
			osDelay(recordTimeBeforeMS);
			if(StepVCNr == 1){
				setValue1 = stepAngles[0];
			}else if(StepVCNr == 2){
				setValue2 = stepAngles[0];
			}
			osDelay(recordTimeAfterMS);
			HAL_TIM_Base_Stop_IT(&htim6); // stop data-logging
			HAL_TIM_Base_Stop_IT(&htim13);

			break;
		case MultiStep:{
			HAL_TIM_Base_Start_IT(&htim13);
			HAL_TIM_Base_Start_IT(&htim6); // start data-logging
			osDelay(recordTimeBeforeMS);
			if(StepVCNr == 1){
				setValue1 = stepAngles[0];
			}else if(StepVCNr == 2){
				setValue2 = stepAngles[0];
			}

			uint8_t stepCount = 0;
			while(stepTimesMs[stepCount] != 0){
				osDelay(stepTimesMs[stepCount]);
				if(StepVCNr == 1){
					setValue1 = stepAngles[stepCount+1];
				}else if(StepVCNr == 2){
					setValue2 = stepAngles[stepCount+1];
				}
				stepCount++;

			}
			osDelay(recordTimeAfterMS);
			HAL_TIM_Base_Stop_IT(&htim6); // stop data-logging
			HAL_TIM_Base_Stop_IT(&htim13);
		}break;
		case SingleDuty:
			HAL_TIM_Base_Stop_IT(&htim14); // disable PIDs
			HAL_TIM_Base_Start_IT(&htim13); // start 10kHz clock
			HAL_TIM_Base_Start_IT(&htim6); // start data-logging
			DRV_X_Axis.enablePWM();
			DRV_Y_Axis.enablePWM();
			osDelay(recordTimeBeforeMS);
			if(StepVCNr == 1){
				DRV_X_Axis.setPWM(stepAngles[0]);
			}else if(StepVCNr == 2){
				DRV_Y_Axis.setPWM(stepAngles[0]);
			}
			osDelay(recordTimeAfterMS);
			DRV_X_Axis.disablePWM();
			DRV_Y_Axis.disablePWM();
			HAL_TIM_Base_Stop_IT(&htim6); // stop data-logging
			HAL_TIM_Base_Stop_IT(&htim13);
			break;
		case Dutysweep:
			HAL_TIM_Base_Stop_IT(&htim14); // disable PIDs
			DRV_X_Axis.enablePWM();
			DRV_Y_Axis.enablePWM();
			HAL_TIM_Base_Start_IT(&htim13); // start 10kHz clock
			HAL_TIM_Base_Start_IT(&htim6); // start data-logging
			osDelay(recordTimeBeforeMS);
			for(int i = 0; i < PIDLIMIT*2; i++){
				if(StepVCNr == 1){
					DRV_X_Axis.setPWM(i-PIDLIMIT);
				}else if(StepVCNr == 2){
					DRV_Y_Axis.setPWM(i-PIDLIMIT);
				}
				osDelay(stepTimesMs[0]);
			}
			osDelay(recordTimeAfterMS);
			HAL_TIM_Base_Stop_IT(&htim6); // stop data-logging
			HAL_TIM_Base_Stop_IT(&htim13);
			DRV_X_Axis.disablePWM();
			DRV_Y_Axis.disablePWM();
			break;
		default:
			break;
	}









    osDelay(1);
  }
  /* USER CODE END startCommandInterpreter */
}








bool setSensorZeroPoint(uint8_t setSensorZero){

	uint16_t sendData[12] = {0};
	uint16_t readData[12] = {0};
	int16_t angleData   = 0;
	uint8_t parity = 0;


	HAL_TIM_Base_Stop_IT(&htim11); // disable SPI readout
	osDelay(1); // wait for SPI transmission to be done

	if(setSensorZero < 1 || setSensorZero > 2) // errorcheck. Has to be Sensor 1 or 2
		return false;


	sendData[setSensorZero - 1] = 0x0016 | 0x4000; // load transmit data with parity and write param
	sendData[setSensorZero + 3] = 0x8017 | 0x4000;
	sendData[setSensorZero + 7] = 0XFFFF;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)sendData,(uint8_t*)readData,12,500); // Set Zero Point register to 0
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);

	angleData = readData[setSensorZero + 9] & 0x3FFF; // clear status and parity




	sendData[setSensorZero + 1] =  angleData > 6 ;
	for(int i = 0; i < 8; i++){
		parity ^= sendData[setSensorZero + 1] >> i; // calculate the even parity (uint16 is half full)
	}
	sendData[setSensorZero + 1] |= (parity << 15); // add parity

	sendData[setSensorZero + 5] =  angleData & 0x003F ;


	parity = 0;
	for(int i = 0; i < 8; i++){
		parity ^= sendData[setSensorZero + 5] >> i; // calculate the even parity (uint16 is half full)
	}
	sendData[setSensorZero + 5] |= (parity << 15); // add parity

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)sendData,(uint8_t*)readData,12,500); // Set Zero Point register to 0
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);

	angleData = readData[setSensorZero + 9] & 0x3FFF; // clear status and parity
	if(angleData > 0x2000) angleData = -((0x3FFF)-angleData);


	uint8_t buf[100] = {0};

//	sprintf(buf,"Angle Sensor %u set to Zero. Latest Data readout: %i",setSensorZero,((angleData * 180) / 0x2000));

//	xQueueSend(xQueueUART3RX,buf,0);

	HAL_TIM_Base_Start_IT(&htim11); // disable SPI readout
}

