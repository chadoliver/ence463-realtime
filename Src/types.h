/*
 * types.h
 *
 *  Created on: Aug 7, 2013
 *      Author: chad_oliver
 */

#ifndef TYPES_H_
#define TYPES_H_

#define TRUE 	(1)
#define FALSE 	(0)

typedef void (*function_ptr)( void * );

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed int int16_t;
typedef unsigned int 	uint16_t;
typedef signed long int int32_t;
typedef unsigned long int uint32_t;
typedef signed long long int int64_t;
typedef unsigned long long int uint64_t;

typedef uint8_t reg_data;

typedef enum {
	ACCEL_X,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z
} MeasurementType;

typedef struct {
	const char* text;
	MeasurementType type;
} ScreenMessage;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} vector3;

typedef struct {
	vector3 accel;
	vector3 gyro;
} MeasurementState;

typedef enum {
	READ,
	WRITE
} I2C_Direction;

#endif /* TYPES_H_ */
