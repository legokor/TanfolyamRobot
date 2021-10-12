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

#define MOT_R 0
#define MOT_L 1

void robotControlInit(Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      UART_HandleTypeDef* usbUart);
void setServoPosition(int8_t position);
uint16_t getUsDistance();
void getColorHsv(ColorHsv* color);
int setMotorSpeed(uint8_t mot_lr, float speed);
uint32_t getEncoderPosition(uint8_t mot_lr);
int uartPrintf(const char *fmt, ...);
void delayMs(uint32_t delay);
int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...);

#endif /* INC_ROBOTCONTROL_H_ */
