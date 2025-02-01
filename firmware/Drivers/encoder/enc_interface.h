/*
 * enc_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "stm32f1xx_hal.h"

/**
 * Resolution of the decoder
 */
typedef enum {
    EncoderResolution_1,  //!< Change the counter on rising edges of A
    EncoderResolution_2,  //!< Change the counter on rising and falling edges of A
    EncoderResolution_4,  //!< Change the counter on rising and falling edges of both A and B
} enc_EncoderResolution;

/**
 * Struct for storage of encoder settings and state
 */
typedef struct {
    GPIO_TypeDef* portA;
    uint16_t pinA;
    GPIO_TypeDef* portB;
    uint16_t pinB;

    enc_EncoderResolution resolution;
    int8_t direction;

    volatile uint8_t initialized;
    volatile int32_t counter;

    TIM_HandleTypeDef* timer;
    uint32_t timerPeriod;
    volatile uint32_t overflowCount;
    volatile uint8_t overflowWasReset;
    uint32_t lastTimerVal;
    uint32_t timerFrequency;

    volatile int32_t countInterval;
    uint16_t maxSpeedCps;
} enc_Encoder;

void enc_init(enc_Encoder* encoder,
                 GPIO_TypeDef* portA, uint16_t pinA,
                 GPIO_TypeDef* portB, uint16_t pinB,
                 enc_EncoderResolution resolution, uint8_t reversed,
                 TIM_HandleTypeDef* intervalTimer, uint32_t timerFrequency, uint16_t maxSpeedCps);

void enc_handlerA(enc_Encoder* encoder);

void enc_handlerB(enc_Encoder* encoder);

void enc_timerOverflowHandler(enc_Encoder* encoder);

int32_t enc_getCounterValue(enc_Encoder* encoder);

int32_t enc_getCountsPerSecond(enc_Encoder* encoder);

float enc_getSpeed(enc_Encoder* encoder);

#endif /* ENCODER_ENCODER_H_ */
