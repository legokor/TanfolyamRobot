/*
 * pwm_pwm.c
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms, dkiovics
 *
 * This driver makes it easier to handle PWM outputs.
 * It only starts the PWM channel and provides functions for setting the output compare register.
 * All other configuration must be done in CubeMX.
 */
#include "pwm_interface.h"
#include <stdlib.h>

/**
 * Initialize the pwm struct
 * @param pwm to be initialized
 * @param timer to be controlled
 * @param timerChannel to be controlled
 * @param timerPeriod counting period of the timer
 * @param outputType which output is used
 * @return 0 on success, -1 on error
 */
int pwm_init(pwm_Pwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel,
            uint16_t timerPeriod, pwm_PwmOutput outputType) {

    if ((timerChannel != TIM_CHANNEL_1) &&
        (timerChannel != TIM_CHANNEL_2) &&
        (timerChannel != TIM_CHANNEL_3) &&
        (timerChannel != TIM_CHANNEL_4)    ) {
        return -1;
    }

    pwm->timer = timer;
    pwm->channel = timerChannel;
    pwm->timerPeriod = timerPeriod;

    if ( (outputType == PwmOutput_P) || (outputType == PwmOutput_PN) ) {
        HAL_TIM_PWM_Start(pwm->timer, pwm->channel);
    } else if ( (outputType == PwmOutput_N) || (outputType == PwmOutput_PN) ) {
        HAL_TIMEx_PWMN_Start(pwm->timer, pwm->channel);
    }

    return 0;
}

/**
 * Set the output compare value for the timer channel
 * @param pwm
 * @param compareValue
 */
void pwm_setCompareValue(pwm_Pwm* pwm, uint16_t compareValue) {
    switch (pwm->channel) {
        case TIM_CHANNEL_1 : pwm->timer->Instance->CCR1 = compareValue; break;
        case TIM_CHANNEL_2 : pwm->timer->Instance->CCR2 = compareValue; break;
        case TIM_CHANNEL_3 : pwm->timer->Instance->CCR3 = compareValue; break;
        case TIM_CHANNEL_4 : pwm->timer->Instance->CCR4 = compareValue; break;
    }
}

/**
 * Set the compare value between 0 and the timer period
 * @param pwm
 * @param dutyCycle in percent
 */
void pwm_setDutyCylePercent(pwm_Pwm* pwm, float dutyCycle) {
    if (dutyCycle > 100) {
        dutyCycle = 100;
    }
    if (dutyCycle < 0) {
        dutyCycle = 0;
    }

    uint16_t compareValue = pwm->timerPeriod * dutyCycle / 100;
    pwm_setCompareValue(pwm, compareValue);
}

/**
 * Set the duty cycle to 0%
 * @param pwm
 */
void pwm_zero(pwm_Pwm* pwm) {
    pwm_setCompareValue(pwm, 0);
}

/**
 * Set the duty cycle to 100%
 * @param pwm
 */
void pwm_max(pwm_Pwm* pwm) {
    pwm_setCompareValue(pwm, pwm->timerPeriod);
}
