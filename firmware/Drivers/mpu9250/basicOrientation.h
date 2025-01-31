/*
 * basicOrientation.h
 *
 *  Created on: Sep 9, 2024
 *      Author: dkiovics
 */

#ifndef MPU9250_BASICORIENTATION_H_
#define MPU9250_BASICORIENTATION_H_

#include "vec3.h"
#include "mpu9250.h"

void updateOrientation(Mpu9250* imu, volatile Orientation* orientation, float dt);

#endif /* MPU9250_BASICORIENTATION_H_ */
