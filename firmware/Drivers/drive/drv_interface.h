/*
 * drv_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef DRIVE_DRV_INTERFACE_H_
#define DRIVE_DRV_INTERFACE_H_

#include "pwm_interface.h"
#include "enc_interface.h"

typedef enum {
    MotorRunMode_Coast,
    MotorRunMode_Brake,
} drv_MotorRunMode;

typedef struct {
    pwm_Pwm pwm1;
    pwm_Pwm pwm2;
    float speed;
    drv_MotorRunMode runMode;
    uint8_t reversed:1;
} drv_Motor;

typedef struct {
    drv_Motor* motor;
    enc_Encoder* encoder;
    volatile float setPoint;
    float prevError;
    float prevSpeed;
    float integrator;
} drv_SpeedControl;

void drv_motorInit(drv_Motor* motor, pwm_Pwm pwm1, pwm_Pwm pwm2, uint8_t reversed);

void drv_motorSetSpeed(drv_Motor* motor, float speed);

void drv_speedControlInit(drv_SpeedControl* sc, drv_Motor* motor, enc_Encoder* encoder);

void drv_speedControlSetSpeed(drv_SpeedControl* sc, float speed);

void drv_speedControlHandler(drv_SpeedControl* sc);

#endif /* DRIVE_DRV_INTERFACE_H_ */
