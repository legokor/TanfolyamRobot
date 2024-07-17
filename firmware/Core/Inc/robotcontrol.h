/*
 * robotcontrol.h
 *
 *  Created on: Nov 22, 2020
 *      Author: ksstms
 */

#ifndef INC_ROBOTCONTROL_H_
#define INC_ROBOTCONTROL_H_

#include <stdint.h>
#include "lcd.h"
#include "servo.h"
#include "ultrasonic.h"
#include "speed_control.h"
#include "encoder.h"
#include "color_sensor.h"
#include "uart.h"
#include "mpu9250.h"

#include "robotcontrol-api.h"

#if US_SENSOR
void robotControlInit(volatile Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      volatile Uart* usbUart, volatile Uart* espUart, Mpu9250* imu);
#elif IR_SENSOR
void robotControlInit(volatile Servo* usServo, volatile InfraRed* irSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      volatile Uart* usbUart, volatile Uart* espUart, Mpu9250* imu);
#else
	#error "No ranging module defined as active"
#endif

#endif /* INC_ROBOTCONTROL_H_ */
