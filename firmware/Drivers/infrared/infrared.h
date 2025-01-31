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
    uint16_t captureStart;
    volatile uint16_t lastDistance;
    uint8_t pwmIsHigh;
} InfraRed;

void irInit(InfraRed* ir, TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequency);
void irHandlerRisingCapture(InfraRed* ir, uint16_t captureVal);
void irHandlerFallingCapture(InfraRed* ir, uint16_t captureVal);
uint16_t irGetDistance(InfraRed* ir);

#endif /* INFRARED_INFRARED_H_ */
