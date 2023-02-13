/*
 * ultrasonic.h
 *
 *  Created on: Nov 3, 2020
 *      Author: ksstms
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
    volatile uint16_t captureStop;
    volatile uint8_t busy;
} UltraSonic;

void usInit(volatile UltraSonic* us, GPIO_TypeDef* triggerPort, uint16_t triggerPin,
            TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequency,
            TIM_HandleTypeDef* delayTimer, uint32_t delayTimerFrequencyHz);
void usHandlerRisingCapture(volatile UltraSonic* us, uint16_t captureVal);
void usHandlerFallingCapture(volatile UltraSonic* us, uint16_t captureVal);
void usStartMeasurement(volatile UltraSonic* us);
int16_t usGetDistance(volatile UltraSonic* us);
int16_t usMeasureBlocking(volatile UltraSonic* us);

#endif /* ULTRASONIC_ULTRASONIC_H_ */
