/*
 * encoder.c
 *
 *  Created on: Nov 2, 2020
 *      Author: ksstms
 *
 *
 * Quadrature encoder waveforms:
 *
 * Direction 1:
 *                 ----+     +-----+     +-----
 *             A       |     |     |     |
 *                     +-----+     +-----+
 *                 -------+     +-----+     +--
 *             B          |     |     |     |
 *                        +-----+     +-----+
 *
 * Direction 2:
 *                 -------+     +-----+     +--
 *             A          |     |     |     |
 *                        +-----+     +-----+
 *                 ----+     +-----+     +-----
 *             B       |     |     |     |
 *                     +-----+     +-----+
 *
 * The decoding can be done in 4 different resolutions:
 *     1X: on rising edge of A: check B and inc/dec counter
 *     2X: same as 1X but on both edges
 *     4X: same as 2X but on both A and B edges
 */
#include "encoder.h"

/**
 * Initialize the encoder struct
 * @param encoder empty encoder struct
 * @param portX GPIO port of encoder terminal X
 * @param pinX GPIO pin of encoder terminal X
 * @param resolution decoding resolution
 * @param direction  1 increase counter when A is leading
 *                  -1 increase counter when B is leading
 */
void encoderInit(volatile Encoder* encoder,
                 GPIO_TypeDef* portA, uint16_t pinA,
                 GPIO_TypeDef* portB, uint16_t pinB,
                 EncoderResolution resolution, int8_t direction) {

    encoder->portA = portA;
    encoder->pinA = pinA;
    encoder->portB = portB;
    encoder->pinB = pinB;
    encoder->resolution = resolution;

    // Default to positive direction
    if (direction != 1 && direction != -1) {
        encoder->direction = 1;
    } else {
        encoder->direction = direction;
    }

    encoder->counter = 0;
}

/**
 * Call this from the GPIO interrupt handler when there's an interrupt on the A terminal
 * @param encoder
 */
void encoderHandlerA(volatile Encoder* encoder) {
    GPIO_PinState risingEdgeA = HAL_GPIO_ReadPin(encoder->portA, encoder->pinA);
    GPIO_PinState stateB = HAL_GPIO_ReadPin(encoder->portB, encoder->pinB);

    if (risingEdgeA) {
        if (stateB == GPIO_PIN_RESET) {
            encoder->counter += encoder->direction;
        } else {
            encoder->counter -= encoder->direction;
        }
    } else {
        if ( (encoder->resolution == EncoderResolution_2) || (encoder->resolution == EncoderResolution_4) ) {
            if (stateB == GPIO_PIN_SET) {
                encoder->counter += encoder->direction;
            } else {
                encoder->counter -= encoder->direction;
            }
        }
    }
}

/**
 * Call this from the GPIO interrupt handler when there's an interrupt on the B terminal
 * @param encoder
 */
void encoderHandlerB(volatile Encoder* encoder) {
    if (encoder->resolution != EncoderResolution_4) {
        return;
    }

    GPIO_PinState risingEdgeB = HAL_GPIO_ReadPin(encoder->portB, encoder->pinB);
    GPIO_PinState stateA = HAL_GPIO_ReadPin(encoder->portA, encoder->pinA);
    if (risingEdgeB) {
        if (stateA == GPIO_PIN_SET) {
            encoder->counter += encoder->direction;
        } else {
            encoder->counter -= encoder->direction;
        }
    } else {
        if (stateA == GPIO_PIN_RESET) {
            encoder->counter += encoder->direction;
        } else {
            encoder->counter -= encoder->direction;
        }
    }
}

uint32_t encoderGetCounterValue(volatile const Encoder* encoder) {
    return encoder->counter;
}
