/*
 * pwm_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
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
} pwm_Pwm;

/**
 * PWM output type
 */
typedef enum {
    PwmOutput_P,   //!< Use the normal output
    PwmOutput_N,   //!< Use the complementary output
    PwmOutput_PN,  //!< Use both outputs
} pwm_PwmOutput;

int pwm_init(pwm_Pwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel,
            uint16_t timerPeriod, pwm_PwmOutput outputType);

void pwm_setCompareValue(pwm_Pwm* pwm, uint16_t compareValue);

void pwm_setDutyCylePercent(pwm_Pwm* pwm, float dutyCycle);

void pwm_zero(pwm_Pwm* pwm);

void pwm_max(pwm_Pwm* pwm);

#endif /* PWM_PWM_H_ */
