/*
 * sleep.c
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#include "FreeRTOS.h"
#include "task.h"

void sleep_ms(int ms) {
	const portTickType delay = ms / portTICK_RATE_MS;
	vTaskDelay(delay);
}

