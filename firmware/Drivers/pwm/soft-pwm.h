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
    TIM_HandleTypeDef* timer;
    uint32_t timerChannel;
    uint32_t timerPeriod;
    GPIO_TypeDef* port;
    uint16_t pin;
    uint16_t pwmPeriod;
    uint16_t compareValue;
    uint8_t pinState;
} SoftPwm;

int softPwmInit(SoftPwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel, uint32_t timerPeriod,
                GPIO_TypeDef* port, uint16_t pin, uint16_t pwmPeriod);
SoftPwm* softPwmCreate(TIM_HandleTypeDef* timer, uint32_t timerChannel, uint32_t timerPeriod,
                       GPIO_TypeDef* port, uint16_t pin, uint16_t pwmPeriod);
void softPwmSetCompareValue(SoftPwm* pwm, uint32_t compareValue);
void softPwmSetDutyCylePercent(SoftPwm* pwm, float dutyCycle);
void softPwmHandler(SoftPwm* pwm);

#endif /* PWM_SOFT_PWM_H_ */
