/*
 * robotcontrol.c
 *
 *  Created on: Nov 22, 2020
 *      Author: ksstms
 */
#include "robotcontrol.h"
#include <stdio.h>
#include <stdarg.h>

static Servo* servo;
static volatile UltraSonic* us;
static volatile ColorSensor* cs;
static volatile SpeedControl* scL;
static volatile SpeedControl* scR;
static volatile Encoder* encoderL;
static volatile Encoder* encoderR;
static UART_HandleTypeDef* uart;

/**
 * Save the pointer to every driver instance used here
 */
void robotControlInit(Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      UART_HandleTypeDef* usbUart) {
    servo = usServo;
    us = usSensor;
    cs = colorSensor;
    scL = scLeft;
    scR = scRight;
    encoderL = encoderLeft;
    encoderR = encoderRight;
    uart = usbUart;
}


void setServoPosition(int8_t position) {
    servoSetPosition(servo, position);
}


uint16_t getUsDistance() {
    return usMeasureBlocking(us);   // TODO: measure automatically, provide a non-blocking measurement function
}


void getColorHsv(Color* color) {
    ColorHsv c;
    colorSensorGetHsv(cs, &c);

    color->h = c.h;
    color->s = c.s;
    color->v = c.v;
}


int setMotorSpeed(uint8_t mot_lr, float speed) {
    if (mot_lr == MOT_L) {
        speedControlSetSpeed(scL, speed);
        return 0;
    }
    if (mot_lr == MOT_R){
        speedControlSetSpeed(scR, speed);
        return 0;
    }

    return -1;
}


uint32_t getEncoderPosition(uint8_t mot_lr) {
    if (mot_lr == MOT_L) {
        return encoderGetCounterValue(scL->encoder);
    }
    if (mot_lr == MOT_R){
    	return encoderGetCounterValue(scR->encoder);
    }
    return 0;
}


int uartPrintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsprintf(str, fmt, args);

    HAL_UART_Transmit_IT(uart, (uint8_t*)str, size);

    return size;
}


void delayMs(uint32_t delay) {
    HAL_Delay(delay);
}


int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[64];
    int size = vsprintf(str, fmt, args);

    if (size <= 0) {
        return size;
    }

    return lcdPuts(row, col, str);
}
