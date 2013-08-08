/*
 * lcd.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: chad_oliver
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "screen.h"
#include "lcd_lld.h"
#include "sleep.h"

#define TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE + 500 )
#define DISPLAY_FREQUENCY			( 1000000 )
#define PRINT_QUEUE_SIZE			( 20 )
#define MAX_STRING_LENGTH			( 25 )

#define LINE_ONE					( 10 )
#define LINE_TWO					( 20 )
#define LINE_THREE					( 30 )

#define COLUMN_ONE					( 0 )
#define COLUMN_TWO					( 50 )

//static xQueueHandle printQueue;
//xQueueSendToBack( printQueue, (void *) &message_text, (portTickType) 0 );
//const unsigned portBASE_TYPE printQueueSize = PRINT_QUEUE_SIZE;
//unsigned portBASE_TYPE printElementSize = sizeof(const char*);
//printQueue = xQueueCreate(PRINT_QUEUE_SIZE, sizeof(const char*));	// Create the queue on which errors will be reported.

void draw_state(MeasurementState* state) {
	char string_buffer[9];

	sprintf(string_buffer, "%d", state->accel.x);
	lcd_draw_string(string_buffer,  COLUMN_ONE, LINE_ONE, 15);

	sprintf(string_buffer, "%d", state->accel.y);
	lcd_draw_string(string_buffer,  COLUMN_ONE, LINE_TWO, 15);

	sprintf(string_buffer, "%d", state->accel.z);
	lcd_draw_string(string_buffer,  COLUMN_ONE, LINE_THREE, 15);

	sprintf(string_buffer, "%d", state->gyro.x);
	lcd_draw_string(string_buffer,  COLUMN_TWO, LINE_ONE, 15);

	sprintf(string_buffer, "%d", state->gyro.y);
	lcd_draw_string(string_buffer,  COLUMN_TWO, LINE_TWO, 15);

	sprintf(string_buffer, "%d", state->gyro.z);
	lcd_draw_string(string_buffer,  COLUMN_TWO, LINE_THREE, 15);
}

void screen_thread( void *pvParameters ) {

	xQueueHandle stateQueue = (xQueueHandle) pvParameters;		// do we need to dereference the pointer?
	MeasurementState state = {{13,5,2}, {1,7,4}};

	lcd_init(1000000);

	while (1) {
		if ( xQueueReceive( stateQueue, ( void * ) &state, 0) ) {
			draw_state(&state);
		}
		sleep_ms(100);
	}
}

