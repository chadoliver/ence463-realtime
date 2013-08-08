/*
 * orientation.c
 *
 *  Created on: Aug 7, 2013
 *      Author: chad_oliver
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "orientation.h"
#include "types.h"
#include "sleep.h"

#define TASK_STACK_SIZE 			( configMINIMAL_STACK_SIZE + 500 )
#define MAX_STRING_LENGTH			( 25 )


void orientation_thread( void *pvParameters ) {

	xQueueHandle stateQueue = (xQueueHandle) pvParameters;
	MeasurementState state = {{13,5,2}, {1,7,4}};

	while (1) {
		state.accel.y++;
		xQueueSendToBack(stateQueue, (void *) &state, 0);
		sleep_ms(200);
	}

}
