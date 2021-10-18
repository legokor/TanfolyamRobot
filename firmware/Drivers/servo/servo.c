/*
 * servo.c
 *
 *  Created on: Nov 5, 2020
 *      Author: ksstms
 *
 * This driver uses the PWM driver to generate the control signal for servo motors.
 * Most servos need a 50 Hz PWM signal with pulse width between 1 and 2 ms.
 * 1 ms sets the position to -90 degrees, and 2 ms to +90 degrees.
 */
#include "servo.h"
#include <stdlib.h>

/**
 * Initialize the servo struct
 *
 * @note compareStart and compareEnd values can be swapped for inverse rotation
 *
 * @param servo to be initialized
 * @param pwm connected to the servo signal
 * @param compareStart timer output compare value for -90 degree position (1 ms)
 * @param compareEnd timer output compare value for +90 degree position (2 ms)
 */
void servoInit(Servo* servo, Pwm* pwm, uint16_t compareStart, uint16_t compareEnd) {
    servo->pwm = pwm;
    servo->compareStart = compareStart;
    servo->compareEnd = compareEnd;

    uint16_t mid = compareStart + (compareEnd - compareStart)/2;

    pwmSetCompareValue(servo->pwm, mid);
}

/**
 * Create a Servo struct. This also creates the Pwm used for the servo control.
 *
 * @note This function allocates memory for the structs!
 * @note compareStart and compareEnd values can be swapped for inverse rotation
 *
 * @param timer used for PWM generation
 * @param timerChannel used for PWM generation
 * @param timerPeriod counting period of the timer
 * @param outputType which pwm output is used (see pwm.h)
 * @param compareStart timer output compare value for -90 degree position (1 ms)
 * @param compareEnd output compare value for +90 degree position (2 ms)
 * @return pointer to the created struct, NULL on error
 */
Servo* servoCreate(TIM_HandleTypeDef* timer, uint32_t timerChannel, uint16_t timerPeriod,
                   PwmOutput outputType, uint16_t compareStart, uint16_t compareEnd) {

    Pwm* pwm = pwmCreate(timer, timerChannel, timerPeriod, outputType);

    if (pwm == NULL) {
        return NULL;
    }

    Servo* servo = (Servo*) malloc(sizeof(Servo));

    if (servo != NULL) {
        servoInit(servo, pwm, compareStart, compareEnd);
    }

    return servo;
}

/**
 * Set the servo's position in degrees
 * @param servo
 * @param position must be between -90 and 90. Otherwise it will be clipped to those values.
 */
void servoSetPosition(Servo* servo, int8_t position) {
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
    uint16_t compareVal = mid + diff;

    pwmSetCompareValue(servo->pwm, compareVal);
}
