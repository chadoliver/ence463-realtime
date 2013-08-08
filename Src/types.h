/*
 * types.h
 *
 *  Created on: Aug 7, 2013
 *      Author: chad_oliver
 */

#ifndef TYPES_H_
#define TYPES_H_

typedef void (*function_ptr)( void * );

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
	long x;
	long y;
	long z;
} Vector3;

typedef struct {
	Vector3 accel;
	Vector3 gyro;
} MeasurementState;

#endif /* TYPES_H_ */
