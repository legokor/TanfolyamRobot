/*
 * us_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef ULTRASONIC_ULTRASONIC_H_
#define ULTRASONIC_ULTRASONIC_H_

#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* triggerPort;
    uint16_t triggerPin;
    TIM_HandleTypeDef* delayTimer;
    TIM_HandleTypeDef* captureTimer;
    uint16_t captureTimerPeriodNs;
    uint16_t delayTimerPeriodNs;
    volatile uint16_t captureStart;
    volatile uint16_t lastDistance;
    volatile uint8_t measurementValid;
    volatile uint8_t echoIsHigh;
    volatile uint8_t pulseActive;
    uint8_t timerCounter;
} us_UltraSonic;

void us_init(us_UltraSonic* us, GPIO_TypeDef* triggerPort, uint16_t triggerPin,
            TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequency,
            TIM_HandleTypeDef* delayTimer, uint32_t delayTimerFrequencyHz);

void us_handlerRisingCapture(us_UltraSonic* us, uint16_t captureVal);

void us_handlerFallingCapture(us_UltraSonic* us, uint16_t captureVal);

void us_startMeasurementPulseAsync(us_UltraSonic* us);

void us_handleCompareAsync(us_UltraSonic* us);

uint16_t us_getDistance(us_UltraSonic* us);

#endif /* ULTRASONIC_ULTRASONIC_H_ */
