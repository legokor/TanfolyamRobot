/*
 * ir_infrared.h
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
} ir_InfraRed;

void ir_init(ir_InfraRed* ir, TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequency);

void ir_handlerRisingCapture(ir_InfraRed* ir, uint16_t captureVal);

void ir_handlerFallingCapture(ir_InfraRed* ir, uint16_t captureVal);

uint16_t ir_getDistance(ir_InfraRed* ir);

#endif /* INFRARED_INFRARED_H_ */
