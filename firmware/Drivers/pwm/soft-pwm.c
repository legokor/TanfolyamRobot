/*
 * soft-pwm.c
 *
 *  Created on: Oct 17, 2021
 *      Author: ksstms
 *
 * This library uses the output compare function of a timer to generate delays for software PWM
 * signal generation. This way the frequency of the PWM can be lower than the timer's period frequency.
 *
 * The softPwmHandler() function should be called from the timer's OC interrupt. The handler toggles
 * the GPIO pin, then sets the Capture Compare Register to delay the next interrupt.
 *
 * Note that the handler function must be able to run between PWM edges, so signals with very low or
 * very high  duty cycles may not be possible to generate. However, the 0% and 100% duty cycle cases
 * are handled here.
 */
#include "soft-pwm.h"

/**
 * Check if the argument is a valid timer channel setting
 * @param timerChannel to be checked
 * @return 0 if invalid
 */
int isValidTimerChannel(uint32_t channel) {
    return ((channel == TIM_CHANNEL_1) ||
            (channel == TIM_CHANNEL_2) ||
            (channel == TIM_CHANNEL_3) ||
            (channel == TIM_CHANNEL_4)   );
}

/**
 * Get a pointer to a timer channel's capture compare register
 * @param timer
 * @param channel
 * @return
 */
volatile uint32_t* getCaptureCompareRegister(TIM_HandleTypeDef* timer, uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1:
            return &timer->Instance->CCR1;

        case TIM_CHANNEL_2:
            return &timer->Instance->CCR2;

        case TIM_CHANNEL_3:
            return &timer->Instance->CCR3;

        case TIM_CHANNEL_4:
            return &timer->Instance->CCR4;

        default:
            return NULL;
    }
}

/**
 * Initialize a SoftPwm struct
 * @param pwm to be initialized
 * @param timer for the OC delay generation
 * @param timerChannel for the OC delay generation
 * @param timerPeriod counter scale of the timer
 * @param port GPIO port of the output signal
 * @param pin GPIO pin of the output signal
 * @param pwmPeriod in timer ticks
 * @param invertedOutput 0 for normal signal, 1 for inverted
 * @return 0 on success
 */
int softPwmInit(SoftPwm* pwm, TIM_HandleTypeDef* timer, uint32_t timerChannel, uint32_t timerPeriod,
                GPIO_TypeDef* port, uint16_t pin, uint16_t pwmPeriod, uint8_t invertedOutput) {
    if (!isValidTimerChannel(timerChannel)) {
        return -1;
    }
    pwm->timer = timer;
    pwm->timerChannel = timerChannel;
    pwm->timerPeriod = timerPeriod;
    pwm->port = port;
    pwm->pin = pin;
    pwm->pwmPeriod = pwmPeriod;

    pwm->compareValue = 0;
    pwm->pinState = 0;
    pwm->invertedOutput = invertedOutput;

    HAL_TIM_OC_Start_IT(timer, timerChannel);

    return 0;
}

/**
 * Create a SoftPwm struct
 * @note This function allocates memory for the struct!
 *
 * @param timer for the OC delay generation
 * @param timerChannel for the OC delay generation
 * @param timerPeriod counter scale of the timer
 * @param port GPIO port of the output signal
 * @param pin GPIO pin of the output signal
 * @param pwmPeriod in timer ticks
 * @param invertedOutput 0 for normal signal, 1 for inverted
 * @return 0 on success
 */
SoftPwm* softPwmCreate(TIM_HandleTypeDef* timer, uint32_t timerChannel, uint32_t timerPeriod,
                       GPIO_TypeDef* port, uint16_t pin, uint16_t pwmPeriod, uint8_t invertedOutput) {
    if (!isValidTimerChannel(timerChannel)) {
        return NULL;
    }
    SoftPwm* pwm = (SoftPwm*) malloc(sizeof(SoftPwm));
    if (pwm != NULL) {
        softPwmInit(pwm, timer, timerChannel, timerPeriod, port, pin, pwmPeriod, invertedOutput);
    }
    return pwm;
}

/**
 * Set the output compare value
 * @param pwm
 * @param compareValue
 */
void softPwmSetCompareValue(SoftPwm* pwm, uint32_t compareValue) {
    if (compareValue > pwm->pwmPeriod) {
        compareValue = pwm->pwmPeriod;
    }

    if (pwm->invertedOutput) {
        pwm->compareValue = pwm->pwmPeriod - compareValue;
    } else {
        pwm->compareValue = compareValue;
    }
}

/**
 * Set the compare value between 0 and the PWM period
 * @param pwm
 * @param dutyCycle in percent
 */
void softPwmSetDutyCylePercent(SoftPwm* pwm, float dutyCycle) {
    if (dutyCycle > 100) {
        dutyCycle = 100;
    }
    if (dutyCycle < 0) {
        dutyCycle = 0;
    }

    softPwmSetCompareValue(pwm, pwm->pwmPeriod * dutyCycle / 100);
}

/**
 * Call this from the timer's output compare interrupt
 * @param pwm
 */
void softPwmHandler(SoftPwm* pwm) {
    // It's volatile to avoid compiler warning
    volatile uint32_t* ccr = getCaptureCompareRegister(pwm->timer, pwm->timerChannel);
    if (ccr == NULL) {
        return;
    }

    // Set output, calculate delay
    uint16_t nextDelay = 0;
    if (pwm->compareValue == pwm->pwmPeriod) {                          // Corner case: 100% duty
        HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_SET);
        pwm->pinState = 1;
    } else if (pwm->compareValue == 0) {                                // Corner case: 0% duty
        HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_RESET);
        pwm->pinState = 0;
    } else {
        if (pwm->pinState == 0) {                                       // Normal operation, rising edge
            HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_SET);
            pwm->pinState = 1;
            nextDelay = pwm->compareValue;
        } else {                                                        // Normal operation, falling edge
            HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_RESET);
            pwm->pinState = 0;
            nextDelay = pwm->pwmPeriod - pwm->compareValue;
        }
    }
    // Set CCR
    uint32_t nextCcr = *ccr + nextDelay;
    if (nextCcr >= pwm->timerPeriod) {
        nextCcr -= pwm->timerPeriod;
    }

    *ccr = nextCcr;
}
