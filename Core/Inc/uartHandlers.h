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
void uartRXHandler(void const * argument);
void uartTXHandler(void const * argument);
void startCommandInterpreter(void const * argument);
bool setSensorZeroPoint(uint8_t setSensorZero);

enum StepTypes { Unkown, SingleStep, MultiStep, SingleDuty, Dutysweep};


#ifdef __cplusplus
}
#endif
#endif /* UARTHANDLERS_H_ */
