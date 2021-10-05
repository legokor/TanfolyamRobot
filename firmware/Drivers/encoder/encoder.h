/*
 * encoder.h
 *
 *  Created on: Nov 2, 2020
 *      Author: ksstms
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "stm32f1xx_hal.h"

#define SPEED_FILTER 16

/**
 * Resolution of the decoder
 */
typedef enum {
    EncoderResolution_1,  //!< Change the counter on rising edges of A
    EncoderResolution_2,  //!< Change the counter on rising and falling edges of A
    EncoderResolution_4,  //!< Change the counter on rising and falling edges of both A and B
} EncoderResolution;

/**
 * Struct for storage of encoder settings and state
 */
typedef struct {
    GPIO_TypeDef* portA;
    uint16_t pinA;
    GPIO_TypeDef* portB;
    uint16_t pinB;

    EncoderResolution resolution;
    int8_t direction;

    uint8_t initialized;
    uint32_t counter;

    TIM_HandleTypeDef* timer;
    uint32_t timerPeriod;
    uint32_t overflowCount;
    uint32_t lastTimerVal;

    uint8_t filterIndex;
    int32_t countInterval[SPEED_FILTER];
    uint16_t maxSpeedCps;
} Encoder;

void encoderInit(volatile Encoder* encoder,
                 GPIO_TypeDef* portA, uint16_t pinA,
                 GPIO_TypeDef* portB, uint16_t pinB,
                 EncoderResolution resolution, uint8_t reversed,
                 TIM_HandleTypeDef* intervalTimer, uint16_t maxSpeedCps);
void encoderHandlerA(volatile Encoder* encoder);
void encoderHandlerB(volatile Encoder* encoder);
void encoderTimerOverflowHandler(Encoder* encoder);
uint32_t encoderGetCounterValue(volatile const Encoder* encoder);
int32_t encoderGetCountsPerSecond(Encoder* encoder);
float encoderGetSpeed(Encoder* encoder);

#endif /* ENCODER_ENCODER_H_ */
