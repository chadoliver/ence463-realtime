/*
 * mpu6050_lld.c
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#include "mpu6050_lld.h"
#include "inc/lm3s9b90.h"                // direct pin control
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "sleep.h"

#define	MAX_TX_BUFFER   	( 50 )
#define	MAX_RX_BUFFER   	( 50 )

#define 	MPU6050_ADDRESS		( 0x68 )
#define	I2C_BASE			I2C1_MASTER_BASE

reg_data txbuf[MAX_TX_BUFFER];          // Transmit buffer
reg_data rxbuf[MAX_RX_BUFFER];          // Receive buffer

static void write_register (mpu6050_regaddr address, reg_data rdata);
static reg_data* read_register(mpu6050_regaddr address, int num_rx_bytes);

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
}

void mpu6050_init(void) {

    reg_data rdata;

    // todo: set up a semaphore.

    i2c_init();		// set the bus speed and enable the master module.

	write_register(MPU6050_PWR_MGMT_1, MPU6050_PM1_RESET);
	sleep_ms(200);  // wait for device reset

	write_register(MPU6050_SIGNAL_PATH_RESET, 0b111);
	sleep_ms(200);  // wait for signal path reset

	rdata = MPU6050_PM1_X_GYRO_CLOCKREF & (~(MPU6050_PM1_SLEEP));   // make sure device is 'awake'
	write_register(MPU6050_PWR_MGMT_1, rdata);

	rdata = 16;                                          // 2 ms sample period.
	write_register(MPU6050_SMPLRT_DIV, rdata);

	rdata = MPU6050_I2C_BYPASS | MPU6050_INT_LEVEL | MPU6050_LATCH_INT_EN;
	write_register(MPU6050_INT_PIN_CFG, rdata);

	//rdata = MPU6050_I2C_MST_EN;
	//mpu6050_write_register(MPU6050_USER_CTRL, rdata);

	rdata = MPU6050_A_HPF_RESET | MPU6050_A_SCALE_pm8g;
	write_register(MPU6050_ACCEL_CONFIG, rdata);

	rdata = MPU6050_G_SCALE_pm500;
	write_register(MPU6050_GYRO_CONFIG, rdata);

	rdata = MPU6050_INT_EN_DATA_RD_EN;
	write_register(MPU6050_INT_ENABLE, rdata);

	//serial.printString("Successfully configured MPU-9150.");

	write_register(MPU6050_USER_CTRL, 0b01000000);

	rdata = *(read_register(MPU6050_USER_CTRL, 1));
	//serial.printInt(rdata);
};


static void write_register(mpu6050_regaddr register_address, reg_data register_value) {

	// In order to write a value to a register, we have to transmit two values: the register address followed by the register value.

	tBoolean isRead	= FALSE;										// we're writing a value, not reading it.
    I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, isRead);		// Set the slave address, and set the Master to Transmit mode.

    I2CMasterDataPut(I2C_BASE, register_address);					// Place the first character to be sent in the data register. In this case, it's the register address.
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);	// Initiate send of character from Master to Slave.

    while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.

    I2CMasterDataPut(I2C_BASE, register_value);						// Send register value.
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);	// Tell peripheral that that's the end of the transmission.

    while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.

    uint32t errors = I2CMasterErr(I2C_BASE);						// Check for errors.
    if (errors != I2C_MASTER_ERR_NONE) {
    	while(1);													// trap thread, to make it easier to debug.
    }
};

static reg_data* temp_read_register(mpu6050_regaddr register_address, int num_rx_bytes) {

	// In order to write a value to a register, we have to transmit two values: the register address followed by the register value.

	tBoolean isRead	= TRUE;											// we're reading a value from a register
    I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, isRead);		// Set the slave address, and set the Master to Receive mode.

    I2CMasterDataPut(I2C_BASE, register_address);					// Place the first character to be sent in the data register. In this case, it's the register address.
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);	// Initiate send of character from Master to Slave.

    while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.

    I2CMasterDataPut(I2C_BASE, register_value);						// Send register value.
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);	// Tell peripheral that that's the end of the transmission.

    while ( I2CMasterBusy(I2C_BASE));								// Delay until transmission completes.

    uint32t errors = I2CMasterErr(I2C_BASE);						// Check for errors.
    if (errors != I2C_MASTER_ERR_NONE) {
    	while(1);													// trap thread, to make it easier to debug.
    }
};

static reg_data* read_register(mpu6050_regaddr address, int num_rx_bytes) {
    msg_t status = RDY_OK;

    txbuf[0] = address;
	i2cAcquireBus(i2c_driver_ptr);
	status = i2cMasterTransmitTimeout(i2c_driver_ptr, getDeviceAddress(), txbuf, 1, rxbuf, num_rx_bytes, i2c_timeout);
	i2cReleaseBus(i2c_driver_ptr);
	printAnyErrors(status);

	printAnyErrors(status);
    return rxbuf;
};

static vector3 mpu6050_read_measurement(mpu6050_regaddr address) {

    vector3 measurement;

    reg_data* received = read_register(address, 6);

    measurement.x = (received[0] << 8) | received[1];
    measurement.y = (received[2] << 8) | received[3];
    measurement.z = (received[4] << 8) | received[5];

    return measurement;
};

vector3 read_accel(void) {
    return read_measurement(MPU6050_ACCEL_XOUT_H);
};

vector3 read_rot(void) {
    return read_measurement(MPU6050_GYRO_XOUT_H);
};
