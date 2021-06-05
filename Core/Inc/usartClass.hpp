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

class uartClass {
public:
	uartClass();

	~uartClass();


    /**
     * @brief sends data put into its queue
     */
    void TxTask();

    /**
     * @brief Starts Tx task
     */
    static void TxTask(void * argument);

    /**
     * @brief gets data from a queue and calls a user callback function
     */
    void RxTask();

    /**
     * @brief Starts Rx task
     */
    static void RxTask(void * argument);


	bool getRxQueueHandle(QueueHandle_t *handle);
	bool getTxQueueHandle(QueueHandle_t *handle);
	bool getTxTaskHandle(TaskHandle_t *handle);
	bool getRxTaskHandle(TaskHandle_t *handle);


private:
	TaskHandle_t m_hTxTask;
	TaskHandle_t m_hRxTask;
	QueueHandle_t m_hTxQueue;
	QueueHandle_t m_hRxQueue;

	void* m_TxTaskCallback;





};

#endif /* INC_USARTCLASS_HPP_ */
