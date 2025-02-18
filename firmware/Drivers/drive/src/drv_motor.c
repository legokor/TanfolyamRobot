/*
 * drv_motor.c
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms, dkiovics
 *
 * Driver for H-bridge motor drivers.
 *
 * The bridge has 2 control inputs and 2 outputs for the motor. It works like this:
 *
 *      IN1   IN2 | OUT1  OUT2 | FUNCTION
 *     -----------|------------|-----------------
 *       0     0  |  Z     Z   | Coast/fast decay
 *       0     1  |  L     H   | Reverse
 *       1     0  |  H     L   | Forward
 *       1     1  |  L     L   | Brake/slow decay
 *
 * Both inputs can be driven with PWM to control the speed of the motor. In slow decay mode the motor
 * position can be controlled more precisely, as it can be braked instantly by stopping the PWM.
 *
 *      IN1   IN2 | FUNCTION
 *     -----------|--------------------
 *      PWM    0  | Forward, fast decay
 *       1    PWM | Forward, slow decay
 *       0    PWM | Reverse, fast decay
 *      PWM    1  | Reverse, slow decay
 *
 * In slow decay/brake mode the PWM duty cycle has to be inverted!
 */
#include "drv_motor.h"
#include "drv_interface.h"
#include <stdlib.h>


/**
 * Initialize the motor struct
 * @param motor to be initialized
 * @param pwm1 drives one input of the bridge
 * @param pwm2 drives the other input of the bridge
 * @param reversed reverse the rotation direction
 */
void drv_motorInit(drv_Motor* motor, pwm_Pwm pwm1, pwm_Pwm pwm2, uint8_t reversed) {
    motor->pwm1 = pwm1;
    motor->pwm2 = pwm2;
    motor->reversed = reversed;
    motor->runMode = MotorRunMode_Brake;
    motor->speed = 0;
    drv_motorSetSpeed(motor, motor->speed);
}

/**
 * Set motor speed and direction
 * @param motor
 * @param speed in percent. This has to be between -100 and +100. Otherwise it will be clipped to those values.
 */
void drv_motorSetSpeed(drv_Motor* motor, float speed) {
    if (speed > 100) {
        speed = 100;
    }
    if (speed < -100) {
        speed = -100;
    }

    float epsilon = 0.01;   // for float comparison

    // Stop
    if (speed > 0-epsilon && speed < 0+epsilon) {
        if (motor->runMode == MotorRunMode_Coast) {
            drv_motorCoast(motor);
        } else {
            drv_motorBrake(motor);
        }
    } else {
        pwm_Pwm* pwmPin;    // This output will have pwm
        pwm_Pwm* fixPin;    // This output will be either high or low

        // Determine which pin is which based on the speed and motor inversion
        if ( (speed > 0 && !motor->reversed) || (speed < 0 && motor->reversed) ) {

            if (motor->runMode == MotorRunMode_Coast) {
                pwmPin = &motor->pwm1;
                fixPin = &motor->pwm2;
            } else {
                pwmPin = &motor->pwm2;
                fixPin = &motor->pwm1;
            }

        } else {
            if (motor->runMode == MotorRunMode_Coast) {
                pwmPin = &motor->pwm2;
                fixPin = &motor->pwm1;
            } else {
                pwmPin = &motor->pwm1;
                fixPin = &motor->pwm2;
            }
        }

        float pwmDutyCycle = (speed > 0) ? speed : -speed;

        if (motor->runMode == MotorRunMode_Coast) {
            pwm_zero(fixPin);
            pwm_setDutyCylePercent(pwmPin, pwmDutyCycle);
        } else {
            pwm_max(fixPin);
            pwm_setDutyCylePercent(pwmPin, 100-pwmDutyCycle);    // invert duty cycle in brake mode
        }
    }

    motor->speed = speed;
}

/**
 * Set coast or brake mode
 * @param motor
 * @param mode
 */
void drv_motorSetRunMode(drv_Motor* motor, drv_MotorRunMode mode) {
    if (motor->runMode == mode) {
        return;
    }

    motor->runMode = mode;
    // Speed has to be reconfigured because the two modes need different setups
    drv_motorSetSpeed(motor, motor->speed);
}

/**
 * Stop the motor in brake mode
 * @param motor
 */
void drv_motorBrake(drv_Motor* motor) {
    pwm_max(&motor->pwm1);
    pwm_max(&motor->pwm2);
}

/**
 * Stop the motor in coast mode
 * @param motor
 */
void drv_motorCoast(drv_Motor* motor) {
    pwm_zero(&motor->pwm1);
    pwm_zero(&motor->pwm2);
}
