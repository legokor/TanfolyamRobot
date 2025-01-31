/*
 * pwm.h
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms
 */

#ifndef PWM_PWM_H_
#define PWM_PWM_H_

#include "stm32f1xx_hal.h"

/**
 * Struct for settings
 */
typedef struct {
    TIM_HandleTypeDef* timer;
    uint32_t channel;
    uint16_t timerPeriod;
} Pwm;

/**
 * PWM output type
 */
typedef enum {
    PwmOutput_P,   //!< Use the normal output
    PwmOutput_N,   //!< Use the complementary output
    PwmOutput_PN,  //!< Use both outputs
} PwmOutput;

int pwmInit(Pwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel,
            uint16_t timerPeriod, PwmOutput outputType);
void pwmSetCompareValue(Pwm* pwm, uint16_t compareValue);
void pwmSetDutyCylePercent(Pwm* pwm, float dutyCycle);
void pwmZero(Pwm* pwm);
void pwmMax(Pwm* pwm);

#endif /* PWM_PWM_H_ */
