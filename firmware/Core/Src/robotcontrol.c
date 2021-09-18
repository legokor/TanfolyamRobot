/*
 * robotcontrol.c
 *
 *  Created on: Nov 22, 2020
 *      Author: ksstms
 */
#include "robotcontrol.h"
#include <stdio.h>
#include <stdarg.h>

Servo* servo;
volatile UltraSonic* us;
volatile ColorSensor* cs;
volatile Motor* motorL;
volatile Motor* motorR;
volatile Encoder* encoderL;
volatile Encoder* encoderR;
UART_HandleTypeDef* uart;

/**
 * Save the pointer to every driver instance used here
 */
void robotControlInit(Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile Motor* motorLeft, volatile Motor* motorRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
                      UART_HandleTypeDef* usbUart) {
    servo = usServo;
    us = usSensor;
    cs = colorSensor;
    motorL = motorLeft;
    motorR = motorRight;
    encoderL = encoderLeft;
    encoderR = encoderRight;
    uart = usbUart;
}

/**
 * Set the servo's position in degrees
 * @param position must be between -90 and 90. Otherwise it will be clipped to those values.
 */
void setServoPosition(int8_t position) {
    servoSetPosition(servo, position);
}

/**
 * Get the distance measured by the ultrasonic ranging sensor
 * @return distance in cm
 */
uint16_t getUsDistance() {
    return usMeasureBlocking(us);   // TODO: measure automatically, provide a non-blocking measurement function
}

/**
 * Read color in HSV format
 * @param color
 */
void getColorHsv(ColorHsv* color) {
    colorSensorGetHsv(cs, color);
}

/**
 * Print to the USB serial port. Works just like regular printf.
 */
int uartPrintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsprintf(str, fmt, args);

    HAL_UART_Transmit_IT(uart, (uint8_t*)str, size);

    return size;
}

/**
 * Do nothing for the specified time
 * @param delay in ms
 */
void delayMs(uint32_t delay) {
    HAL_Delay(delay);
}

/**
 * Works just like printf after the position specifiers
 * @param row of starting position
 * @param col of starting position
 * @param fmt printf-like format string followed by a variable number of arguments
 */
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
