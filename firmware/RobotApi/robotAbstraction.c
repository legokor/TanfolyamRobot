/*
 * robotcontrol.c
 *
 *  Created on: Nov 22, 2020
 *      Author: ksstms
 */
#include "robotAbstraction-api.h"
#include "robotInstance.h"
#include "telemetry.h"
#include "lcd.h"

#include <stdio.h>
#include <stdarg.h>


extern main_Telemetry main_telemetry;


void setServoPosition(int8_t position) {
    servoSetPosition(&main_robotInstance.servo, position);
}

#if US_SENSOR
uint16_t getUsDistance() {
    return usGetDistance(&main_robotInstance.us);
}
#elif IR_SENSOR
uint16_t getIrDistance() {
    return irGetDistance(&main_robotInstance.ir);
}
#else
	#error "No ranging module defined as active"
#endif


void getColorHsv(Color* color) {
    ColorHsv c;
    colorSensorGetHsv(&main_robotInstance.colorSensor, &c);

    color->h = c.h;
    color->s = c.s;
    color->v = c.v;
}


int setMotorSpeed(uint8_t mot_lr, float speed) {
    if (mot_lr == MOT_L) {
        speedControlSetSpeed(&main_robotInstance.leftSpeedCtrl, speed);
        return 0;
    }
    if (mot_lr == MOT_R){
        speedControlSetSpeed(&main_robotInstance.rightSpeedCtrl, speed);
        return 0;
    }

    return -1;
}


int getEncoderPosition(uint8_t mot_lr) {
    if (mot_lr == MOT_L) {
        return encoderGetCounterValue(&main_robotInstance.leftEncoder);
    }
    if (mot_lr == MOT_R){
    	return encoderGetCounterValue(&main_robotInstance.rightEncoder);
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

    uart_transmit(&main_robotInstance.usbUart, str);

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

    main_sendTextToEsp(&main_telemetry, &main_robotInstance, str);

    return 0;
}

int espRead(char* data) {
    return uart_receive(&main_robotInstance.espUart, data);
}

Vec3 getAccData(){
	return mpu9250_readAccData(&main_robotInstance.imu);
}

Vec3 getGyroData(){
	return mpu9250_readGyroData(&main_robotInstance.imu);
}

Vec3 getMagData(){
	return mpu9250_readMagData(&main_robotInstance.imu);
}

float getTempData(){
	return mpu9250_readTempData(&main_robotInstance.imu);
}

Orientation getOrientation(){
	return main_robotInstance.orientation;
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
