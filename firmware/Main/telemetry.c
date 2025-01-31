/*
 * telemetry.c
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#include "telemetry.h"
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>


extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* SERVER_IP;


void main_telemetryInit(main_Telemetry* telemetry)
{
	telemetry->prevData[0] = 0;
	telemetry->sendDataEnabled = 1;
}

void main_sendDataToEsp(main_Telemetry* telemetry, main_RobotInstance* robotInstance)
{
	if(!telemetry->sendDataEnabled)
		return;
	uint8_t r, g, b;
	colorSensorGetRgb(&robotInstance->colorSensor, &r, &g, &b);
	int cps1 = encoderGetCountsPerSecond(&robotInstance->rightEncoder);
	int cps2 = encoderGetCountsPerSecond(&robotInstance->leftEncoder);
	int cnt1 = encoderGetCounterValue(&robotInstance->rightEncoder);
	int cnt2 = encoderGetCounterValue(&robotInstance->leftEncoder);

	Vec3 acc = mpu9250_readAccData(&robotInstance->imu);
	Vec3 gyro = mpu9250_readGyroData(&robotInstance->imu);
	Vec3 mag = mpu9250_readMagData(&robotInstance->imu);
	float temp = mpu9250_readTempData(&robotInstance->imu);

	char data[256];
	//D: R G B Setpoint1 Setpoint2 CPS1 CPS2 CNT1 CNT2 Servo US/IR ACC_X ACC_Y ACC_Z GYRO_X GYRO_Y GYRO_Z MAG_X MAG_Y MAG_Z TEMP
	//For some reason when sprintf is called from an interrupt handler it cannot contain any %f formatted values
	//otherwise it sometimes produces garbage output
	sprintf((char*)telemetry->prevData, "D: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", (int)r, (int)g, (int)b,
			(int)robotInstance->rightSpeedCtrl.setPoint, (int)robotInstance->leftSpeedCtrl.setPoint,
			cps1, cps2, cnt1, cnt2, robotInstance->servo.position,
#if US_SENSOR
			robotInstance->us.lastDistance,
#elif IR_SENSOR
			robotInstance->ir.lastDistance/10,
#else
	#error "No ranging module defined as active"
#endif
			(int)(acc.x * 100), (int)(acc.y * 100), (int)(acc.z * 100), (int)gyro.x, (int)gyro.y, (int)gyro.z,
			(int)(mag.x * 10), (int)(mag.y * 10), (int)(mag.z * 10), (int)(temp * 10), (int)(robotInstance->orientation.pitch * 10), (int)(robotInstance->orientation.roll * 10));

	strcpy(data, (char*)telemetry->prevData);
	strcat(data, "\n");

	uart_transmit(&robotInstance->espUart, data);
}

void main_sendTextToEsp(main_Telemetry* telemetry, main_RobotInstance* robotInstance, const char* text)
{
	char data[512];
	telemetry->sendDataEnabled = 0;
	sprintf(data, "%s %s\n", telemetry->prevData, text);
	telemetry->sendDataEnabled = 1;
	uart_transmit(&robotInstance->espUart, data);
}

void main_sendConfigToEsp(main_RobotInstance* robotInstance)
{
	char data[256];
	sprintf(data, "C:\t%s\t%s\t%s\n", WIFI_SSID, WIFI_PASSWORD, SERVER_IP);
	uart_transmit(&robotInstance->espUart, data);
}
