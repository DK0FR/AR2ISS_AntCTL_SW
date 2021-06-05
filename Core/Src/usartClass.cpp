/*
 * usartClass.cpp
 *
 *  Created on: 30.05.2021
 *      Author: Soeren
 */

#include "usartClass.hpp"


uartClass::uartClass(){

}

uartClass::~uartClass(){

}



void uartClass::TxTask(void *argument)
{
  // start task
  static_cast<uartClass*>(argument)->TxTask();
}

void uartClass::RxTask(void *argument)
{
  // start task
  static_cast<uartClass*>(argument)->RxTask();
}


void uartClass::mutable
