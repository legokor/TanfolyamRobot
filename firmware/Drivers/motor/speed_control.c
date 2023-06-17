/*
 * speed_control.c
 *
 *  Created on: Sep 20, 2021
 *      Author: ksstms
 *
 * This driver implements a PI controller for controlling the speed of an encoder motor.
 */
#include "speed_control.h"
#include <stdlib.h>
#include <string.h>

// TODO: put these somewhere else
#define K_P 15.0
#define K_I 15.0
#define DT 0.001
#define I_LIM_MAX  1
#define I_LIM_MIN -1

/**
 * Initialize the speed control struct
 * @param sc to be initialized
 * @param motor to be controlled
 * @param encoder for speed measurement
 */
void speedControlInit(SpeedControl* sc, Motor* motor, Encoder* encoder) {
    sc->motor = motor;
    sc->encoder = encoder;
    sc->setPoint = 0;
    sc->prevError = 0;
    sc->prevSpeed = 0;
    sc->integrator = 0;
}

/**
 * Create and initialize a speed control struct
 * @note This function allocates memory for the struct!
 *
 * @param motor to be controlled
 * @param encoder for speed measurement
 * @return pointer to created and initialized struct, NULL on error
 */
SpeedControl* speedControlCreate(Motor* motor, Encoder* encoder) {
    SpeedControl* sc = (SpeedControl*) malloc(sizeof(SpeedControl));
    if (sc == NULL) {
        return NULL;
    }
    speedControlInit(sc, motor, encoder);
    return sc;
}

/**
 * Chane the setpoint of the controller
 * @param sc
 * @param speed between -100 and 100
 */
void speedControlSetSpeed(SpeedControl* sc, float speed) {
    sc->setPoint = speed;
    motorSetSpeed(sc->motor, speed);
}

/**
 * PI controller function. Call this from a timer interrupt handler.
 * @param sc
 */
void speedControlHandler(SpeedControl* sc) {

//    // Don't bother calculating stuff for stopped motor.
//    if ((sc->setPoint < 1) && (sc->setPoint > -1)) {
//        motorSetSpeed(sc->motor, 0);
//        return;
//    }
//
//    // Calculate error
//    float speed = encoderGetSpeed(sc->encoder);
//    float error = sc->setPoint - speed;
//
//    // Proportional part
//    float proportional = K_P * error;
//
//    // Integral part
//    sc->integrator = sc->integrator + 0.5f * K_I * DT * (error + sc->prevError);
//
//    // Integrator anti-windup
//    if (sc->integrator > I_LIM_MAX) {
//        sc->integrator = I_LIM_MAX;
//    } else if (sc->integrator < I_LIM_MIN) {
//        sc->integrator = I_LIM_MIN;
//    }
//
//    // Set output
//    float out = proportional + sc->integrator;
//    motorSetSpeed(sc->motor, out);
}

