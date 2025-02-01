/*
 * main_interface.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

#include "enc_interface.h"
#if US_SENSOR
#include "us_interface.h"
#elif IR_SENSOR
#include "ir_interface.h"
#endif
#include "srv_interface.h"
#include "txt_interface.h"
#include "us_interface.h"
#include "mpu_interface.h"
#include "clr_interface.h"
#include "drv_interface.h"

typedef struct RobotInstance
{
	clr_ColorSensor colorSensor;
	enc_Encoder rightEncoder, leftEncoder;
	drv_Motor rightMotor, leftMotor;
	drv_SpeedControl rightSpeedCtrl, leftSpeedCtrl;
	srv_Servo servo;
	txt_Uart espUart, usbUart;
	mpu_Mpu9250 imu;
	volatile mpu_Orientation orientation;

	volatile uint8_t initCplt;
	volatile uint8_t espReady;

#if US_SENSOR
	us_UltraSonic us;
#elif IR_SENSOR
	ir_InfraRed ir;
#endif
} main_RobotInstance;

typedef struct
{
	volatile char prevData[256];
	volatile uint8_t sendDataEnabled;
} main_Telemetry;


extern main_RobotInstance main_robotInstance;
extern main_Telemetry main_telemetry;


void main_initRobot(void);

void main_runRobot(void);

void main_sendTextToEsp(main_Telemetry* telemetry, main_RobotInstance* robotInstance, const char* text);

void main_delayUs(uint32_t us);

#endif /* MAIN_INTERFACE_H_ */
