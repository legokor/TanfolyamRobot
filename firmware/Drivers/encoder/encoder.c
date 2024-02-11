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
#include <string.h>

// If the count does not change for this many timer overflows, we can assume that the motor is stopped.
#define SPEED_TICK_TIMEOUT 500

typedef enum {
    nop,
    inc,
    dec
} CounterOperation;

/**
 * Initialize the encoder struct
 * @param encoder empty encoder struct
 * @param portX GPIO port of encoder terminal X
 * @param pinX GPIO pin of encoder terminal X
 * @param resolution decoding resolution
 * @param reversed 0 will increase counter when A is leading, otherwise increase counter when B is leading
 * @param intervalTimer timer used for speed measurement
 * @param maxSpeedCps maximum speed in counts/sec.
 */
void encoderInit(volatile Encoder* encoder,
                 GPIO_TypeDef* portA, uint16_t pinA,
                 GPIO_TypeDef* portB, uint16_t pinB,
                 EncoderResolution resolution, uint8_t reversed,
                 TIM_HandleTypeDef* intervalTimer, uint16_t maxSpeedCps) {

    encoder->portA = portA;
    encoder->pinA = pinA;
    encoder->portB = portB;
    encoder->pinB = pinB;
    encoder->resolution = resolution;

    if (reversed) {
        encoder->direction = -1;
    } else {
        encoder->direction =  1;
    }

    encoder->counter = 0;

    encoder->timer = intervalTimer;
    encoder->timerPeriod = intervalTimer->Init.Period;
    encoder->overflowCount = 0;
    encoder->overflowWasReset = 1;
    encoder->lastTimerVal = 0;
    encoder->countInterval = 0;
    encoder->maxSpeedCps = maxSpeedCps / 100;

    encoder->initialized = 1;
}

/**
 * Call this from the GPIO interrupt handler when there's an interrupt on the A terminal
 * @param encoder
 */
void encoderHandlerA(volatile Encoder* encoder) {
    if (!encoder->initialized) {
        return;
    }

    // Get timer values
    uint32_t timerVal = encoder->timer->Instance->CNT;
    uint32_t overflowCount = encoder->overflowCount;

    encoder->overflowCount = 0;

	// Calculate elapsed time since last update
	int32_t elapsed = overflowCount * encoder->timerPeriod + timerVal - encoder->lastTimerVal;

	encoder->lastTimerVal = timerVal;

	GPIO_PinState risingEdgeA = HAL_GPIO_ReadPin(encoder->portA, encoder->pinA);
	GPIO_PinState stateB = HAL_GPIO_ReadPin(encoder->portB, encoder->pinB);

	CounterOperation co = nop;

	if (risingEdgeA) {
		if (stateB == GPIO_PIN_RESET) {
			co = inc;
		} else {
			co = dec;
		}
	} else {
		if ( (encoder->resolution == EncoderResolution_2) || (encoder->resolution == EncoderResolution_4) ) {
			if (stateB == GPIO_PIN_SET) {
				co = inc;
			} else {
				co = dec;
			}
		}
	}

	if (co == nop) {
		return;
	}
	if (co == inc) {
		encoder->counter += encoder->direction;
	} else if (co == dec) {
		encoder->counter -= encoder->direction;
		elapsed = -elapsed;
	}

	if(encoder->overflowWasReset){
	    encoder->overflowWasReset = 0;
	}else{
		encoder->countInterval = elapsed;
	}
}

/**
 * Call this from the GPIO interrupt handler when there's an interrupt on the B terminal
 * @param encoder
 */
void encoderHandlerB(volatile Encoder* encoder) {
    if (!encoder->initialized) {
        return;
    }

    if (encoder->resolution != EncoderResolution_4) {
        return;
    }

    // Get timer values
    uint32_t timerVal = encoder->timer->Instance->CNT;
    uint32_t overflowCount = encoder->overflowCount;

    encoder->overflowCount = 0;

    // Calculate elapsed time since last update
    int32_t elapsed = overflowCount * encoder->timerPeriod + timerVal - encoder->lastTimerVal;

    encoder->lastTimerVal = timerVal;

    GPIO_PinState risingEdgeB = HAL_GPIO_ReadPin(encoder->portB, encoder->pinB);
    GPIO_PinState stateA = HAL_GPIO_ReadPin(encoder->portA, encoder->pinA);

    CounterOperation co = nop;

    if (risingEdgeB) {
        if (stateA == GPIO_PIN_SET) {
            co = inc;
        } else {
            co = dec;
        }
    } else {
        if (stateA == GPIO_PIN_RESET) {
            co = inc;
        } else {
            co = dec;
        }
    }

    if (co == nop) {
        return;
    }
    if (co == inc) {
        encoder->counter += encoder->direction;
    } else if (co == dec) {
        encoder->counter -= encoder->direction;
        elapsed = -elapsed;
    }

    if(encoder->overflowWasReset){
		encoder->overflowWasReset = 0;
	}else{
		encoder->countInterval = elapsed;
	}
}

/**
 * Call this when the timer used for speed calculation overflows
 * @param encoder
 */
void encoderTimerOverflowHandler(Encoder* encoder) {
    if (!encoder->initialized) {
        return;
    }

    encoder->overflowCount++;
    if (encoder->overflowCount == SPEED_TICK_TIMEOUT) {
        encoder->countInterval = 0;
        encoder->overflowCount = 0;
        encoder->overflowWasReset = 1;
    }
}

/**
 * Read the current position of the encoder
 * @param encoder
 * @return counter value
 */
int32_t encoderGetCounterValue(volatile const Encoder* encoder) {
    return encoder->counter;
}

/**
 * Read the speed in counts/sec
 * @param encoder
 * @return speed in CPS
 */
int32_t encoderGetCountsPerSecond(Encoder* encoder) {
    __disable_irq();
    int32_t counts = encoder->countInterval;
    __enable_irq();

    if (encoder->direction == -1) {
    	counts = -counts;
    }

    if (counts == 0) {
        return 0;
    }

    return 64000000.0f / counts;
}

/**
 * Read the scaled speed of the encoder.
 * If maxSpeedCps is set correctly, this should be between -100 and +100.
 * @param encoder
 * @return scaled speed
 */
float encoderGetSpeed(Encoder* encoder) {
    float cps = encoderGetCountsPerSecond(encoder);
    return cps / encoder->maxSpeedCps;
}
