/*
 * infrared.h
 *
 *  Created on: Apr 15, 2024
 *      Author: serbanadi
 */

#ifndef INFRARED_INFRARED_H_
#define INFRARED_INFRARED_H_

#include "stm32f1xx_hal.h"

typedef struct {
    TIM_HandleTypeDef* captureTimer;
    uint16_t captureTimerPeriodNs;
    volatile uint16_t captureStart;
    volatile uint16_t lastDistance;
    volatile uint8_t pwmIsHigh;
} InfraRed;

void irInit(volatile InfraRed* ir, TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequency);
void irHandlerRisingCapture(volatile InfraRed* ir, uint16_t captureVal);
void irHandlerFallingCapture(volatile InfraRed* ir, uint16_t captureVal);
uint16_t irGetDistance(volatile InfraRed* ir);

#endif /* INFRARED_INFRARED_H_ */
