/*
 * usartClass.hpp
 *
 *  Created on: 30.05.2021
 *      Author: Soeren
 */

#ifndef INC_USARTCLASS_HPP_
#define INC_USARTCLASS_HPP_

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32l5xx_hal.h"

struct TaskParams{
	uint16_t stackSize;
	uint8_t prio;
	uint8_t queueLength;
	uint16_t queueElementLength; // set this to 0, if the dynamic UartHeapStruct is to be used
};

struct LedsSetup{
	GPIO_TypeDef* RXPORT;
	GPIO_TypeDef* TXPORT;
	uint16_t 	RXPIN;
	uint16_t 	TXPIN;
};

typedef struct {
	uint8_t* bufPtr;
	bool usedInRTOS;
	uint16_t size;
} UartHeapStruct;


class usartClass {
public:
//	usartClass();
	usartClass();
	~usartClass();

	void initTasks(TaskParams rxTask, TaskParams txTask, UART_HandleTypeDef *huart, void (*rxCallback)(UART_HandleTypeDef*, uint8_t*, uint16_t));
	void setupLeds(LedsSetup leds);


    /**
     * @brief Starts Tx task
     */
    static void TxTask(void * argument);
    /**
     * @brief sends data put into its queue
     */
    void TxTask();

    virtual void TxTaskStatic();

    virtual void TxTaskDynamic();


    /**
     * @brief Starts Rx task
     */
    static void RxTask(void * argument);

    /**
     * @brief gets data from a queue and calls a user callback function
     */
    virtual void RxTask();


    void sendToRxQueueFromISR(uint8_t sendData, BaseType_t* higherPrio );

    void rxCpltCallback();

    void txCpltCallback();



    QueueHandle_t getRxQueueHandle();
    QueueHandle_t getTxQueueHandle();
    TaskHandle_t getTxTaskHandle();
    TaskHandle_t getRxTaskHandle();
    uint16_t getRxReceiveSize();

    uint16_t getTxQueueSize();




    UART_HandleTypeDef* getRxUartHandle();



private:
    uint8_t* m_rxDMABuffer;


	TaskHandle_t 	m_hTxTask;
	TaskHandle_t 	m_hRxTask;
	QueueHandle_t 	m_hTxQueue;
	QueueHandle_t 	m_hRxQueue;
	LedsSetup 		m_Leds;
	bool 			m_LedsOn;
	TaskParams 		m_txTaskParam;
	TaskParams 		m_rxTaskParam;

	void  (*m_TxTaskCallback) ( UART_HandleTypeDef * huart, uint8_t *data, uint16_t length);


	UART_HandleTypeDef* m_huart;





};

#endif /* INC_USARTCLASS_HPP_ */
