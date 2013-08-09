/*
 * i2c_wrapper.h
 *
 *  Created on: Aug 9, 2013
 *      Author: chad_oliver
 */

#ifndef I2C_WRAPPER_H_
#define I2C_WRAPPER_H_

#include "types.h"
#include "driverlib/i2c.h"

void i2c_init(void);

void i2c_set_direction(I2C_Direction direction);

void i2c_send_with_mode(reg_data data, uint8_t mode);

reg_data i2c_read_with_mode(uint8_t mode);

void i2c_check_for_errors(void);

#endif /* I2C_WRAPPER_H_ */
