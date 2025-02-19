/*
 * drv_speedControl.c
 *
 *  Created on: Sep 20, 2021
 *      Author: ksstms
 *
 * This driver implements a PI controller for controlling the speed of an encoder motor.
 */
#include "drv_interface.h"
#include <stdlib.h>
#include <string.h>


#define K_P 15.0
#define K_I 20.0
#define DT 0.001
#define I_LIM_MAX  70
#define I_LIM_MIN -70

/**
 * Initialize the speed control struct
 * @param sc to be initialized
 * @param motor to be controlled
 * @param encoder for speed measurement
 */
void drv_speedControlInit(drv_SpeedControl* sc, drv_Motor* motor, enc_Encoder* encoder) {
    sc->motor = motor;
    sc->encoder = encoder;
    sc->setPoint = 0;
    sc->prevError = 0;
    sc->prevSpeed = 0;
    sc->integrator = 0;
}

/**
 * Chane the setpoint of the controller
 * @param sc
 * @param speed between -100 and 100
 */
void drv_speedControlSetSpeed(drv_SpeedControl* sc, float speed) {
    sc->setPoint = speed;
}

/**
 * PI controller function. Call this from a timer interrupt handler.
 * @param sc
 */
void drv_speedControlHandler(drv_SpeedControl* sc) {

    // Don't bother calculating stuff for stopped motor.
    if ((sc->setPoint < 1) && (sc->setPoint > -1)) {
        drv_motorSetSpeed(sc->motor, 0);
        return;
    }

    // Calculate error
    float speed = enc_getSpeed(sc->encoder);
    float error = sc->setPoint - speed;

    // Proportional part
    float proportional = K_P * error;

    // Integral part
    sc->integrator = sc->integrator + 0.5f * K_I * DT * error;

    // Integrator anti-windup
    if (sc->integrator > I_LIM_MAX) {
        sc->integrator = I_LIM_MAX;
    } else if (sc->integrator < I_LIM_MIN) {
        sc->integrator = I_LIM_MIN;
    }

    // Set output
    float out = proportional + sc->integrator;
    drv_motorSetSpeed(sc->motor, out);
}

