/*
 * robotInstance.h
 *
 *  Created on: Jan 28, 2025
 *      Author: dkiovics
 */

#ifndef ROBOTINSTANCE_H_
#define ROBOTINSTANCE_H_

#include "color_sensor.h"
#include "encoder.h"
#include "infrared.h"
#include "motor.h"
#include "servo.h"
#include "uart.h"
#include "ultrasonic.h"
#include "mpu9250.h"
#include "vec3.h"
#include "speed_control.h"

typedef struct RobotInstance
{
	ColorSensor colorSensor;
	Encoder rightEncoder, leftEncoder;
	Motor rightMotor, leftMotor;
	SpeedControl rightSpeedCtrl, leftSpeedCtrl;
	Servo servo;
	Uart espUart, usbUart;
	Mpu9250 imu;
	volatile Orientation orientation;

	volatile uint8_t initCplt;
	volatile uint8_t espReady;

#if US_SENSOR
	UltraSonic us;
#elif IR_SENSOR
	InfraRed ir;
#endif
} main_RobotInstance;

extern main_RobotInstance main_robotInstance;

#endif /* ROBOTINSTANCE_H_ */
