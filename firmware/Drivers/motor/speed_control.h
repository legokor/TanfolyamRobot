/*
 * speed_control.h
 *
 *  Created on: Sep 20, 2021
 *      Author: ksstms
 */

#ifndef MOTOR_SPEED_CONTROL_H_
#define MOTOR_SPEED_CONTROL_H_

#include "motor.h"
#include "encoder.h"

#define ENCODER_SPEED_DELAY 10

typedef struct {
    Motor* motor;
    Encoder* encoder;

    float setPoint;
    float prevError;
    float prevSpeed;
    float integrator;
} SpeedControl;

void speedControlInit(SpeedControl* sc, Motor* motor, Encoder* encoder);
SpeedControl* speedControlCreate(Motor* motor, Encoder* encoder);
void speedControlSetSpeed(SpeedControl* sc, float speed);
void speedControlHandler(SpeedControl* sc);

#endif /* MOTOR_SPEED_CONTROL_H_ */
