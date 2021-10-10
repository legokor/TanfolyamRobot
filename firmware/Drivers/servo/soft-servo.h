/*
 * soft-servo.h
 *
 *  Created on: Oct 10, 2021
 *      Author: ksstms
 */

#ifndef SERVO_SOFT_SERVO_H_
#define SERVO_SOFT_SERVO_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
	uint32_t cntr;
	uint32_t period;
	uint32_t start;
	uint32_t end;
	uint32_t compareVal;
} SoftServo;

void softServoInit(SoftServo* servo, GPIO_TypeDef* port, uint16_t pin,
		           uint32_t period, uint32_t start, uint32_t end);
void softServoSetPosition(SoftServo* servo, int8_t position);
void softServoHandler(SoftServo* servo);


#endif /* SERVO_SOFT_SERVO_H_ */
