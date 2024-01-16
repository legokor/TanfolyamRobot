/*
 * robotcontrol.c
 *
 *  Created on: Nov 22, 2020
 *      Author: ksstms
 */
#include "robotcontrol.h"
#include <stdio.h>
#include <stdarg.h>

static volatile Servo* servo;
static volatile UltraSonic* us;
static volatile ColorSensor* cs;
static volatile SpeedControl* scL;
static volatile SpeedControl* scR;
static volatile Encoder* encoderL;
static volatile Encoder* encoderR;
static volatile Uart* uart1;
static volatile Uart* uart3;

/**
 * Save the pointer to every driver instance used here
 */
void robotControlInit(volatile Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
					  volatile Uart* usbUart, volatile Uart* espUart) {
    servo = usServo;
    us = usSensor;
    cs = colorSensor;
    scL = scLeft;
    scR = scRight;
    encoderL = encoderLeft;
    encoderR = encoderRight;
    uart1 = usbUart;
    uart3 = espUart;
}


void setServoPosition(int8_t position) {
    servoSetPosition(servo, position);
}


uint16_t getUsDistance() {
    return usGetDistance(us);
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
    if(size <= 0)
    	return -1;
    str[size] = '\0';

    uart_transmit(uart1, str);

    return 0;
}

/*
int espPrintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsprintf(str, fmt, args);
    if(size <= 0)
    	return;
    str[size] = '\0';

    uart_transmit(uart3, str);

    return 0;
}
*/

int espRead(char* data) {
    return uart_receive(uart3, data);
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
