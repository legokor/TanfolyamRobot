/*
 * soft-servo.c
 *
 *  Created on: Oct 10, 2021
 *      Author: ksstms
 *
 * This driver uses the software-PWM driver to generate the control signal for servo motors.
 * Most servos need a 50 Hz PWM signal with pulse width between 1 and 2 ms.
 * 1 ms sets the position to -90 degrees, and 2 ms to +90 degrees.
 *
 * The handler function shall be called from a timer interrupt. The period property shall be
 * set to the timer interrupt frequency / 50 Hz. The start and end properties shall be set
 * to about period/20 and period/10, but they might need some tweaking to work properly.
 */
#include "soft-servo.h"

/**
 * Initialize a servo struct
 * @note compareStart and compareEnd values can be swapped for inverse rotation
 * @param servo to be initialized
 * @param pwm soft PWM generating the output signal
 * @param compareStart output compare value for -90 degree position (1 ms)
 * @param compareEnd output compare value for +90 degree position (2 ms)
 */
void softServoInit(SoftServo* servo, SoftPwm* pwm, uint32_t compareStart, uint32_t compareEnd) {
    servo->pwm = pwm;
    servo->compareStart = compareStart;
    servo->compareEnd = compareEnd;

    uint32_t mid = (compareStart + compareEnd) / 2;

    softPwmSetCompareValue(servo->pwm, mid);
}

/**
 * Create a Servo struct. This also creates the Pwm used for the servo control.
 * @note This function allocates memory for the structs!
 * @note compareStart and compareEnd values can be swapped for inverse rotation
 *
 * @param port GPIO port of the output signal
 * @param pin GPIO pin of the output signal
 * @param period of the PWM's internal counter
 * @param compareStart output compare value for -90 degree position (1 ms)
 * @param compareEnd output compare value for +90 degree position (2 ms)
 * @return pointer to the created struct, NULL on error
 */
SoftServo* softServoCreate(GPIO_TypeDef* port, uint16_t pin, uint32_t period,
                           uint16_t compareStart, uint16_t compareEnd) {

    SoftPwm* pwm = softPwmCreate(port, pin, period);
    if (pwm == NULL) {
        return NULL;
    }

    SoftServo* servo = (SoftServo*) malloc(sizeof(SoftServo));

    if (servo != NULL) {
        softServoInit(servo, pwm, compareStart, compareEnd);
    }

    return servo;
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

    int32_t s = servo->compareStart;
    int32_t e = servo->compareEnd;

    uint16_t mid = s + (e-s)/2;
    int32_t diff = (e-mid) * position / 90;

    uint32_t compareVal = mid + diff;

    softPwmSetCompareValue(servo->pwm, compareVal);
}

/**
 * Call this from a timer interrupt
 * @param servo
 */
void softServoHandler(SoftServo* servo) {
    softPwmHandler(servo->pwm);
}
