/*
 * uartHandler.h
 *
 *  Created on: 20.03.2019
 *      Author: talruso
 */

#ifndef UARTHANDLERS_H_
#define UARTHANDLERS_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "stdbool.h"

void uartRXHandler(void const * argument);
void uartTXHandler(void const * argument);



#define UARTRXBUFFERSIZE 500
#define UARTTXBUFFERSIZE 500


typedef struct {
	uint8_t* bufPtr;
	bool usedInRTOS;
	uint16_t size;
} UartTXStruct;



enum StepTypes { Unkown, SingleStep, MultiStep, SingleDuty, Dutysweep};


#ifdef __cplusplus
}
#endif
#endif /* UARTHANDLERS_H_ */
