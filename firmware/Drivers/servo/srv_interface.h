/*
 * srv_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "pwm_interface.h"

/**
 * Struct for settings
 */
typedef struct {
    pwm_Pwm pwm;
    uint16_t compareStart;
    uint16_t compareEnd;
    int8_t position;
} srv_Servo;

void srv_init(srv_Servo* servo, pwm_Pwm pwm, uint16_t compareStart, uint16_t compareEnd, int8_t position);

void srv_setPosition(srv_Servo* servo, int8_t position);

#endif /* SERVO_SERVO_H_ */
