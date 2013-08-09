/*
 * mpu6050_lld.c
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#include "mpu6050_lld.h"
#include "inc/hw_types.h"
#include "sleep.h"
#include "i2c_wrapper.h"
#include "mpu6050_register_map.h"

#define	MAX_RX_BUFFER   	( 50 )

reg_data rxbuf[MAX_RX_BUFFER];

void write_register(mpu6050_regaddr register_address, reg_data register_value) {

	// In order to write a value to a register, we have to transmit two values: the register address followed by the register value.

	i2c_set_direction(WRITE);
	i2c_send_with_mode(register_address, I2C_MASTER_CMD_BURST_SEND_START);
	i2c_send_with_mode(register_value,   I2C_MASTER_CMD_BURST_SEND_FINISH);

	i2c_check_for_errors();
};

reg_data* read_register(mpu6050_regaddr register_address, int num_rx_bytes) {

	uint8_t i = 0;

	// write the register address to the device
	i2c_set_direction(WRITE);
	i2c_send_with_mode(register_address, I2C_MASTER_CMD_SINGLE_SEND);


	// now read the values which the slave sends to the microcontroller.
	i2c_set_direction(READ);
	if (num_rx_bytes == 1) {
		rxbuf[0] = i2c_read_with_mode(I2C_MASTER_CMD_SINGLE_RECEIVE);
	}
	else {
		rxbuf[0] = i2c_read_with_mode(I2C_MASTER_CMD_BURST_RECEIVE_START);
		for (i=1; i<(num_rx_bytes-1); i++) {
			rxbuf[i] = i2c_read_with_mode(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		}
		rxbuf[i] = i2c_read_with_mode(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	}

	i2c_check_for_errors();

    return rxbuf;
};

void mpu6050_init(void) {

    reg_data rdata;

    // todo: set up a semaphore.

    i2c_init();

	write_register(MPU6050_PWR_MGMT_1, MPU6050_PM1_RESET);
	sleep_ms(200);  // wait for device reset

	write_register(MPU6050_SIGNAL_PATH_RESET, 7);
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

	write_register(MPU6050_USER_CTRL, 64);		// 0b01000000

	rdata = *(read_register(MPU6050_USER_CTRL, 1));
	//serial.printInt(rdata);
};

vector3 mpu6050_read_measurement(mpu6050_regaddr address) {

    vector3 measurement;

    reg_data* received = read_register(address, 6);

    measurement.x = (received[0] << 8) | received[1];
    measurement.y = (received[2] << 8) | received[3];
    measurement.z = (received[4] << 8) | received[5];

    return measurement;
};

MeasurementState mpu6050_read_state(void) {
	MeasurementState state;
    state.accel = mpu6050_read_measurement(MPU6050_ACCEL_XOUT_H);
    state.gyro = mpu6050_read_measurement(MPU6050_GYRO_XOUT_H);
    return state;
};
