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
#if US_SENSOR
static volatile UltraSonic* us;
#elif IR_SENSOR
static volatile InfraRed* ir;
#else
	#error "No ranging module defined as active"
#endif
static volatile ColorSensor* cs;
static volatile SpeedControl* scL;
static volatile SpeedControl* scR;
static volatile Encoder* encoderL;
static volatile Encoder* encoderR;
static volatile Uart* uart1;
static volatile Uart* uart3;
static Mpu9250* imu;
static volatile Orientation* r_orientation;

/**
 * Save the pointer to every driver instance used here
 */
#if US_SENSOR
void robotControlInit(volatile Servo* usServo, volatile UltraSonic* usSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
					  volatile Uart* usbUart, volatile Uart* espUart, Mpu9250* imuIn, volatile Orientation* orientation) {
#elif IR_SENSOR
void robotControlInit(volatile Servo* usServo, volatile InfraRed* irSensor, volatile ColorSensor* colorSensor,
                      volatile SpeedControl* scLeft, volatile SpeedControl* scRight,
                      volatile Encoder* encoderLeft, volatile Encoder* encoderRight,
					  volatile Uart* usbUart, volatile Uart* espUart, Mpu9250* imuIn, volatile Orientation* orientation) {
#else
	#error "No ranging module defined as active"
#endif
    servo = usServo;
#if US_SENSOR
    us = usSensor;
#elif IR_SENSOR
    ir = irSensor;
#else
	#error "No ranging module defined as active"
#endif
    cs = colorSensor;
    scL = scLeft;
    scR = scRight;
    encoderL = encoderLeft;
    encoderR = encoderRight;
    uart1 = usbUart;
    uart3 = espUart;
    imu = imuIn;
    r_orientation = orientation;
}


void setServoPosition(int8_t position) {
    servoSetPosition(servo, position);
}

#if US_SENSOR
uint16_t getUsDistance() {
    return usGetDistance(us);
}
#elif IR_SENSOR
uint16_t getIrDistance() {
    return irGetDistance(ir);
}
#else
	#error "No ranging module defined as active"
#endif


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


int getEncoderPosition(uint8_t mot_lr) {
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


int espPrintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsprintf(str, fmt, args);
    if(size <= 0)
    	return -1;
    str[size] = '\0';

    for(int p = 0; p < size; p++){
    	if(str[p] == '\n')
    		str[p] = ' ';
    }

    uart_sendTextToEsp(uart3, str);

    return 0;
}

int espRead(char* data) {
    return uart_receive(uart3, data);
}

Vec3 getAccData(){
	return mpu9250_readAccData(imu);
}

Vec3 getGyroData(){
	return mpu9250_readGyroData(imu);
}

Vec3 getMagData(){
	return mpu9250_readMagData(imu);
}

float getTempData(){
	return mpu9250_readTempData(imu);
}

Orientation getOrientation(){
	return *r_orientation;
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
