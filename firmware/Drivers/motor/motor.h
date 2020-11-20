/*
 * motor.h
 *
 *  Created on: Nov 4, 2020
 *      Author: ksstms
 */

#ifndef MOTOR_MOTOR_H_
#define MOTOR_MOTOR_H_

#include "pwm.h"

/**
 * See H-bridge theory of operation
 */
typedef enum {
    MotorRunMode_Coast,//!< MotorRunMode_Coast use coast/fast decay mode
    MotorRunMode_Brake,//!< MotorRunMode_Brake use brake/slow decay mode
} MotorRunMode;

/**
 * Store resources and settings needed by the driver
 */
typedef struct {
    Pwm* pwm1;
    Pwm* pwm2;
    float speed;
    MotorRunMode runMode;
    uint8_t reversed:1;
} Motor;

void motorInit(Motor* motor, Pwm* pwm1, Pwm* pwm2, uint8_t reversed);
Motor* motorCreate(TIM_HandleTypeDef* pwm1Timer, uint32_t pwm1TimerChannel, uint16_t pwm1TimerPeriod, PwmOutput pwm1OutputType,
                   TIM_HandleTypeDef* pwm2Timer, uint32_t pwm2TimerChannel, uint16_t pwm2TimerPeriod, PwmOutput pwm2OutputType,
                   uint8_t reversed);
void motorSetSpeed(Motor* motor, float speed);
void motorSetRunMode(Motor* motor, MotorRunMode mode);
void motorBrake(Motor* motor);
void motorCoast(Motor* motor);

#endif /* MOTOR_MOTOR_H_ */
