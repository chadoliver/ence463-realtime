/*
 * hookFunctions.c
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#include "hookFunctions.h"

void vApplicationIdleHook( void ) {

}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void ) {

}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void ) {
    /* This function will only be called if an API call to create a task, queue
    or semaphore fails because there is too little heap RAM remaining. */
    while(1);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
    /* This function will only be called if a task overflows its stack.  Note
    that stack overflow checking does slow down the context switch
    implementation. */
    while(1);
}
