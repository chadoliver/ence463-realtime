/*
 * mpu6050_lld.h
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#ifndef MPU6050_LLD_H_
#define MPU6050_LLD_H_

#include "types.h"
#include "mpu6050_register_map.h"






vector3 read_measurement (uint8_t address);

void init(I2CDriver* i2c_instance);

void printAnyErrors (msg_t status);
void configure_device(void);
void write_register (uint8_t address, reg_data rdata);
reg_data* read_register(uint8_t address, int rx_bytes);



void configure_device(void);

void write_register (mpu6050_regaddr address, reg_data rdata);
reg_data* read_register (mpu6050_regaddr address, int rx_bytes);
vector3 read_accel (void);
vector3 read_rot (void);

#endif /* MPU6050_LLD_H_ */
