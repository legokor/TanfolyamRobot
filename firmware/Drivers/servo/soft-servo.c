/*
 * soft-servo.c
 *
 *  Created on: Oct 10, 2021
 *      Author: ksstms
 *
 * Software PWM generator for controlling servo motors.
 * Most servos need a 50 Hz PWM signal with pulse width between 1 and 2 ms.
 * 1 ms sets the position to -90 degrees, and 2 ms to +90 degrees.
 *
 * The handler function shall be called from a timer interrupt.
 * The period property shall be set to the timer interrupt frequency / 50 Hz.
 * The start and end properties shall be set to about period/20 and period/10,
 * but they might need some tweaking to work properly.
 */
#include "soft-servo.h"

/**
 * Initialize a servo struct
 * @note compareStart and compareEnd values can be swapped for inverse rotation
 * @param servo to be initialized
 * @param port GPIO port of the control signal
 * @param pin GPIO pin of the control signal
 * @param period of the internal counter
 * @param start counter value for one endpoint
 * @param end counter value for the other endpoint
 */
void softServoInit(SoftServo* servo, GPIO_TypeDef* port, uint16_t pin,
		           uint32_t period, uint32_t start, uint32_t end) {
	servo->port = port;
	servo->pin = pin;
	servo->cntr = 0;
	servo->period = period;
	servo->start = start;
	servo->end = end;
	servo->compareVal = (start + end) / 2;
}

/**
 * Set the servo's position in degrees
 * @param servo
 * @param position must be between -90 and 90. Otherwise, it will be clipped to those values.
 */
void softServoSetPosition(SoftServo* servo, int8_t position) {
    if (position > 90) {
        position = 90;
    }
    if (position < -90) {
        position = -90;
    }

    int32_t s = servo->start;
    int32_t e = servo->end;

    uint16_t mid = s + (e-s)/2;
    int32_t diff = (e-mid) * position / 90;

    servo->compareVal = mid + diff;
}

/**
 * Call this from a timer interrupt
 * @param servo
 */
void softServoHandler(SoftServo* servo) {
	servo->cntr++;

	if (servo->cntr == servo->period) {
		servo->cntr = 0;
	}

	if (servo->cntr < servo->compareVal) {
		HAL_GPIO_WritePin(servo->port, servo->pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(servo->port, servo->pin, GPIO_PIN_RESET);
	}
}
