/*
 * vec3.h
 *
 *  Created on: Jul 7, 2024
 *      Author: dkiovics
 */

#ifndef MPU9250_VEC3_H_
#define MPU9250_VEC3_H_

/**
 * @brief Data storage struct for a float32 Vec3.
 *
 */
typedef struct {
	float x, y, z;
} Vec3;

typedef struct {
	float pitch;
	float roll;
} Orientation;

#endif /* MPU9250_VEC3_H_ */
