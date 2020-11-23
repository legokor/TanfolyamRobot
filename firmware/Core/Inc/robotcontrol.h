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
#include "motor.h"
#include "encoder.h"
#include "color_sensor.h"

void robotControlInit(Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile Motor* motorLeft, volatile Motor* motorRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      UART_HandleTypeDef* usbUart);
void setServoPosition(int8_t position);
uint16_t getUsDistance();
// TODO: color getter
// TODO: motor control
int uartPrintf(const char *fmt, ...);
void delayMs(uint32_t delay);

#endif /* INC_ROBOTCONTROL_H_ */
