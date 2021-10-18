/*
 * servo.h
 *
 *  Created on: Nov 5, 2020
 *      Author: ksstms
 */

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "pwm.h"

/**
 * Struct for settings
 */
typedef struct {
    Pwm* pwm;
    uint16_t compareStart;
    uint16_t compareEnd;
} Servo;

void servoInit(Servo* servo, Pwm* pwm, uint16_t compareStart, uint16_t compareEnd);
Servo* servoCreate(TIM_HandleTypeDef* timer, uint32_t timerChannel, uint16_t timerPeriod,
                   PwmOutput outputType, uint16_t compareStart, uint16_t compareEnd);
void servoSetPosition(Servo* servo, int8_t position);

#endif /* SERVO_SERVO_H_ */
