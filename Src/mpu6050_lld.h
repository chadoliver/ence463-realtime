/*
 * mpu6050_lld.h
 *
 *  Created on: Aug 8, 2013
 *      Author: chad_oliver
 */

#ifndef MPU6050_LLD_H_
#define MPU6050_LLD_H_

#include "types.h"

void mpu6050_init(void);
MeasurementState mpu6050_read_state(void);

#endif /* MPU6050_LLD_H_ */
