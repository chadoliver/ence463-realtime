/*
 * i2c_wrapper.c
 *
 *  Created on: August 9, 2013
 *      Author: chad_oliver
 */

//#include "inc/lm3s9b90.h"                // direct pin control
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "interrupt.h"
#include "gpio.h"
#include "sysctl.h"
#include "i2c_wrapper.h"

#define 	MPU6050_ADDRESS		( 0x68 )
#define	I2C_BASE			I2C1_MASTER_BASE

void i2c_init(void) {

    // I2C1 is not configured for I2C as default
    // -> GPIOPinConfigure() is needed
    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);

    // Setup clock for 20Mhz - commented out because have something similar in main.c
    // SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Turn on I2C1 and reset to a known state
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    // Configure the PortB pins for I2C1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    // Set GPIO Pins for Open-Drain operation
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    // Give control to the I2C Module
    GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);
    GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);

    // Enable and Initialize Master module and Master Clock using 100kbps
    I2CMasterInitExpClk(I2C1_MASTER_BASE, SysCtlClockGet(), FALSE);
};

void i2c_set_direction(I2C_Direction direction) {
	I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, direction);
};

void i2c_send_with_mode(reg_data data, uint8_t mode) {

	I2CMasterDataPut(I2C_BASE, data);
	I2CMasterControl(I2C_BASE, mode);

	while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.
};

reg_data i2c_read_with_mode(uint8_t mode) {

	reg_data data = I2CMasterDataGet(I2C_BASE);
	I2CMasterControl(I2C_BASE, mode);

	while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.
	return data;
};

void i2c_check_for_errors(void) {
	uint32_t errors = I2CMasterErr(I2C_BASE);						// Check for errors.
	if (errors != I2C_MASTER_ERR_NONE) {
		while(1);													// trap thread, to make it easier to debug.
	}
};


