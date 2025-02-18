/*
 * api_robotAbstraction.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#include "api_robotAbstraction.h"
#include "main_interface.h"
#include "tel_interface.h"
#include "lcd_interface.h"

#include <stdio.h>
#include <stdarg.h>


void setServoPosition(int8_t position) 
{
    srv_setPosition(&main_robotInstance.servo, position);
}

#if US_SENSOR
uint16_t getUsDistance() 
{
    return us_getDistance(&main_robotInstance.us);
}
#elif IR_SENSOR
uint16_t getIrDistance() 
{
    return ir_getDistance(&main_robotInstance.ir);
}
#else
	#error "No ranging module defined as active"
#endif


void getColorHsv(Color* color) 
{
    clr_ColorHsv c;
    clr_getHsv(&main_robotInstance.colorSensor, &c);

    color->h = c.h;
    color->s = c.s;
    color->v = c.v;
}


int setMotorSpeed(uint8_t mot_lr, float speed) 
{
    if (mot_lr == MOT_L) {
        drv_speedControlSetSpeed(&main_robotInstance.leftSpeedCtrl, speed);
        return 0;
    }
    if (mot_lr == MOT_R){
        drv_speedControlSetSpeed(&main_robotInstance.rightSpeedCtrl, speed);
        return 0;
    }

    return -1;
}


int getEncoderPosition(uint8_t mot_lr) 
{
    if (mot_lr == MOT_L) {
        return enc_getCounterValue(&main_robotInstance.leftEncoder);
    }
    if (mot_lr == MOT_R){
    	return enc_getCounterValue(&main_robotInstance.rightEncoder);
    }
    return 0;
}


int uartPrintf(const char *fmt, ...) 
{
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsnprintf(str, 256, fmt, args);
    if(size <= 0)
    	return -1;

    txt_transmit(&main_robotInstance.usbUart, str, size);

    return 0;
}


int espPrintf(const char *fmt, ...) 
{
    va_list args;
    va_start(args, fmt);

    char str[256];
    int size = vsnprintf(str, 256, fmt, args);
    if(size <= 0)
    	return -1;

    tel_sendTextToEsp(str);

    return 0;
}

int espRead(char* data) 
{
    return txt_receive(&main_robotInstance.espUart, data);
}

void getAccData(Vec3* acc) 
{
    mpu_Vec3 a = mpu_readAccData(&main_robotInstance.imu);
    acc->x = a.x;
    acc->y = a.y;
    acc->z = a.z;
}

void getGyroData(Vec3* gyro) 
{
    mpu_Vec3 g = mpu_readGyroData(&main_robotInstance.imu);
    gyro->x = g.x;
    gyro->y = g.y;
    gyro->z = g.z;
}

void getMagData(Vec3* mag) 
{
    mpu_Vec3 m = mpu_readMagData(&main_robotInstance.imu);
    mag->x = m.x;
    mag->y = m.y;
    mag->z = m.z;
}

float getTempData()
{
	return mpu_readTempData(&main_robotInstance.imu);
}

void getOrientation(Orientation* orientation) 
{
    orientation->pitch = main_robotInstance.orientation.pitch;
    orientation->roll = main_robotInstance.orientation.roll;
}

void delayMs(uint32_t delay) 
{
    HAL_Delay(delay);
}

void delayUs(uint32_t delay)
{
	main_delayUs(delay);
}

uint32_t getTimeMs()
{
	return HAL_GetTick();
}

int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...) 
{
    va_list args;
    va_start(args, fmt);

    char str[64];
    int size = vsprintf(str, fmt, args);

    if (size <= 0) {
        return size;
    }

    return lcd_puts(row, col, str);
}

void turnRobot(int degrees, int power)
{
	float w = 0;
	if (degrees > 0)
	{
		setMotorSpeed(MOT_L, -power);
		setMotorSpeed(MOT_R, power);
		while(w < degrees)
		{
			w += mpu_readGyroData(&main_robotInstance.imu).z * 0.01f;
			delayUs(10000);
		}
	}
	else
	{
		setMotorSpeed(MOT_L, power);
		setMotorSpeed(MOT_R, -power);
		while(w > degrees)
		{
			w += mpu_readGyroData(&main_robotInstance.imu).z * 0.01f;
			delayUs(10000);
		}
	}
	setMotorSpeed(MOT_L, 0);
	setMotorSpeed(MOT_R, 0);
}
