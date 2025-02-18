/*
 * drv_motor.h
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms, dkiovics
 */

#ifndef DRIVE_MOTOR_H_
#define DRIVE_MOTOR_H_

#include "drv_interface.h"

void drv_motorSetRunMode(drv_Motor* motor, drv_MotorRunMode mode);

void drv_motorBrake(drv_Motor* motor);

void drv_motorCoast(drv_Motor* motor);

#endif /* DRIVE_MOTOR_H_ */
