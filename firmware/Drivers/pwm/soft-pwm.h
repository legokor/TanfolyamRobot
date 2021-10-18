/*
 * soft-pwm.h
 *
 *  Created on: Oct 17, 2021
 *      Author: ksstms
 */

#ifndef PWM_SOFT_PWM_H_
#define PWM_SOFT_PWM_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t cntr;
    uint32_t period;
    uint32_t compareVal;
} SoftPwm;

void softPwmInit(SoftPwm* pwm, GPIO_TypeDef* port, uint16_t pin, uint32_t period);
SoftPwm* softPwmCreate(GPIO_TypeDef* port, uint16_t pin, uint32_t period);
void softPwmSetCompareValue(SoftPwm* pwm, uint32_t compareValue);
void softPwmSetDutyCylePercent(SoftPwm* pwm, float dutyCycle);
void softPwmHandler(SoftPwm* pwm);

#endif /* PWM_SOFT_PWM_H_ */
