/*
 * hookFunctions.h: Hook functions that can get called by the kernel.
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#ifndef HOOKFUNCTIONS_H_
#define HOOKFUNCTIONS_H_

#include "FreeRTOS.h"
#include "task.h"

void vApplicationIdleHook( void );
void vApplicationTickHook( void );
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
void vApplicationTickHook( void );

#endif /* HOOKFUNCTIONS_H_ */
