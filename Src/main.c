//*****************************************************************************
//
// main.c - FreeRTOS porting example on CCS4
//
// Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 5570 of the EK-LM3S8962 Firmware Package.
//
//*****************************************************************************

#include <stdio.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//#include "pin.hpp"

/* Standard Stellaris includes */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#include "hookFunctions.h"
#include "types.h"
#include "screen.h"
#include "orientation.h"

//*****************************************************************************
//
// The speed of the processor clock, which is therefore the speed of the clock
// that is fed to the peripherals.
//
//*****************************************************************************
unsigned long g_ulSystemClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

void configure_mcu(void) {
	// If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
	// a workaround to allow the PLL to operate reliably.
	if (REVISION_IS_A2) {
		SysCtlLDOSet(SYSCTL_LDO_2_75V);
	}

	// Set the clocking to run at 50MHz from the PLL.
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_8MHZ);

	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();
}

int main(void) {

	configure_mcu();

	xQueueHandle stateQueue = xQueueCreate( 20, sizeof(MeasurementState) );

	xTaskCreate(screen_thread,
				"Screen Thread",
				configMINIMAL_STACK_SIZE + 500,
				( void * ) stateQueue,
				1,
				NULL);

	xTaskCreate(orientation_thread,
				"Orientation Thread",
				configMINIMAL_STACK_SIZE + 500,
				( void * ) stateQueue,
				1,
				NULL);

    /* Start the scheduler so our tasks start executing. */
    vTaskStartScheduler();

    /* If all is well we will never reach here as the scheduler will now be
    running.  If we do reach here then it is likely that there was insufficient
    heap available for the idle task to be created. */
    while (1) {};
}





