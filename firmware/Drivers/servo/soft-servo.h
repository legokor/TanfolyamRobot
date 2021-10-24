/*
 * soft-servo.h
 *
 *  Created on: Oct 10, 2021
 *      Author: ksstms
 */

#ifndef SERVO_SOFT_SERVO_H_
#define SERVO_SOFT_SERVO_H_

#include "soft-pwm.h"

/**
 * Struct for settings
 */
typedef struct {
    SoftPwm* pwm;
	uint32_t compareStart;
	uint32_t compareEnd;
} SoftServo;

void softServoInit(SoftServo* servo, SoftPwm* pwm, uint32_t compareStart, uint32_t compareEnd);
SoftServo* softServoCreate(SoftPwm* pwm, uint16_t compareStart, uint16_t compareEnd);
void softServoSetPosition(SoftServo* servo, int8_t position);
void softServoHandler(SoftServo* servo);


#endif /* SERVO_SOFT_SERVO_H_ */
