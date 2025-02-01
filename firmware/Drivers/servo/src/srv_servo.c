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
#include "srv_interface.h"
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
void srv_init(srv_Servo* servo, pwm_Pwm pwm, uint16_t compareStart, uint16_t compareEnd, int8_t position) {
    servo->pwm = pwm;
    servo->compareStart = compareStart;
    servo->compareEnd = compareEnd;
    servo->position = position;

    srv_setPosition(servo, position);
}

/**
 * Set the servo's position in degrees
 * @param servo
 * @param position must be between -90 and 90. Otherwise it will be clipped to those values.
 */
void srv_setPosition(srv_Servo* servo, int8_t position) {
    if (position > 90) {
        position = 90;
    }
    if (position < -90) {
        position = -90;
    }
    servo->position = position;

    int32_t s = servo->compareStart;
    int32_t e = servo->compareEnd;

    uint16_t mid = s + (e-s)/2;
    int32_t diff = (e-mid) * position / 90;
    uint16_t compareVal = mid + diff;

    pwm_setCompareValue(&servo->pwm, compareVal);
}
