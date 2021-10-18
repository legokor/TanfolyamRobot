/*
 * pwm.c
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms
 *
 * This driver makes it easier to handle PWM outputs.
 * It only starts the PWM channel and provides functions for setting the output compare register.
 * All other configuration must be done in CubeMX.
 */
#include "pwm.h"
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
int pwmInit(Pwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel,
            uint16_t timerPeriod, PwmOutput outputType) {

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
        HAL_TIM_PWM_Start_IT(pwm->timer, pwm->channel);                             // TODO: maybe use the period interrupt insted of the pulse finished interrupt
    } else if ( (outputType == PwmOutput_N) || (outputType == PwmOutput_PN) ) {
        HAL_TIMEx_PWMN_Start_IT(pwm->timer, pwm->channel);                          // TODO: maybe use the period interrupt insted of the pulse finished interrupt
    }

    return 0;
}

/**
 * Create a Pwm struct
 * @note This function allocates memory for the struct!
 *
 * @param timer to be controlled
 * @param timerChannel to be controlled
 * @param timerPeriod counting period of the timer
 * @param outputType which output is used
 * @return pointer to the created struct, NULL on error
 */
Pwm* pwmCreate(TIM_HandleTypeDef* timer, uint32_t timerChannel, uint16_t timerPeriod, PwmOutput outputType) {
    if ((timerChannel != TIM_CHANNEL_1) &&
        (timerChannel != TIM_CHANNEL_2) &&
        (timerChannel != TIM_CHANNEL_3) &&
        (timerChannel != TIM_CHANNEL_4)    ) {
        return NULL;
    }

    Pwm* pwm = (Pwm*) malloc(sizeof(Pwm));
    if (pwm != NULL) {
        pwmInit(pwm, timer, timerChannel, timerPeriod, outputType);
    }

    return pwm;
}

/**
 * Set the output compare value for the timer channel
 * @param pwm
 * @param compareValue
 */
void pwmSetCompareValue(Pwm* pwm, uint16_t compareValue) {
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
void pwmSetDutyCylePercent(Pwm* pwm, float dutyCycle) {
    if (dutyCycle > 100) {
        dutyCycle = 100;
    }
    if (dutyCycle < 0) {
        dutyCycle = 0;
    }

    uint16_t compareValue = pwm->timerPeriod * dutyCycle / 100;
    pwmSetCompareValue(pwm, compareValue);
}

/**
 * Set the duty cycle to 0%
 * @param pwm
 */
void pwmZero(Pwm* pwm) {
    pwmSetCompareValue(pwm, 0);
}

/**
 * Set the duty cycle to 100%
 * @param pwm
 */
void pwmMax(Pwm* pwm) {
    pwmSetCompareValue(pwm, pwm->timerPeriod);
}
