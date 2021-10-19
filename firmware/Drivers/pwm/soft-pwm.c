/*
 * soft-pwm.c
 *
 *  Created on: Oct 17, 2021
 *      Author: ksstms
 *
 * Software controlled PWM library.
 * The SoftPwm struct has an internal counter, which is incremented when the handler is called.
 * If the counter is less than the compare value, the GPIO pin is set to 1, otherwise it's set to 0.
 */
#include "soft-pwm.h"

/**
 * Initialize a SoftPwm struct
 * @param pwm to be initialized
 * @param port GPIO port of the output signal
 * @param pin GPIO pin of the output signal
 * @param period of the internal counter
 */
void softPwmInit(SoftPwm* pwm, GPIO_TypeDef* port, uint16_t pin, uint32_t period) {
    pwm->port = port;
    pwm->pin = pin;
    pwm->cntr = 0;
    pwm->period = period;
    pwm->compareVal = 0;
}

/**
 * Create a SoftPwm struct
 * @note This function allocates memory for the struct!
 * @param port GPIO port of the output signal
 * @param pin GPIO pin of the output signal
 * @param period of the internal counter
 * @return pointer to the created struct, NULL on error
 */
SoftPwm* softPwmCreate(GPIO_TypeDef* port, uint16_t pin, uint32_t period) {
    SoftPwm* pwm = (SoftPwm*) malloc(sizeof(SoftPwm));
    if (pwm != NULL) {
        softPwmInit(pwm, port, pin, period);
    }
    return pwm;
}

/**
 * Set the output compare value
 * @param pwm
 * @param compareValue
 */
void softPwmSetCompareValue(SoftPwm* pwm, uint32_t compareValue) {
    pwm->compareVal = compareValue;
}

/**
 * Set the compare value between 0 and the timer period
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

    pwm->compareVal = pwm->period * dutyCycle / 100;
}

/**
 * Call this from a timer interrupt
 * @param pwm
 */
void softPwmHandler(SoftPwm* pwm) {
    pwm->cntr++;

    if (pwm->cntr >= pwm->period) {
        pwm->cntr = 0;
    }

    if (pwm->port == NULL) {
        return;
    }

    if (pwm->cntr < pwm->compareVal) {
        HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(pwm->port, pwm->pin, GPIO_PIN_RESET);
    }
}
