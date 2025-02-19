/*
 * telemetry.c
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#include "main_interface.h"
#include "tel_interface.h"
#include "tel_telemetry.h"
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

static volatile uint8_t tel_espConfigCplt = 0;
static volatile uint8_t tel_sendDataEnabled = 0;

#define TEL_MAX_DATA_SIZE 		200
#define TEL_MAX_PACKET_SIZE 	280

static char tel_dataOut[TEL_MAX_DATA_SIZE];
static char tel_packetData[TEL_MAX_PACKET_SIZE];
static size_t tel_packetSize = 0;

static_assert(sizeof(tel_OutputData) + 2 <= TEL_MAX_DATA_SIZE, "tel_OutputData size is too big for the data buffer");

uint8_t tel_initEsp()
{
	size_t size = snprintf(tel_dataOut, TEL_MAX_DATA_SIZE, "C\t%s\t%s\t%s", WIFI_SSID, WIFI_PASSWORD, SERVER_IP);
	if (size >= TEL_MAX_DATA_SIZE)
		return 0;
	if (!tel_convertDataToPacket(size))
		return 0;
	txt_transmit(&main_robotInstance.espUart, tel_packetData, tel_packetSize);
	tel_espConfigCplt = 1;
	tel_sendDataEnabled = 1;
	return 1;
}

uint8_t tel_convertDataToPacket(size_t size)
{
	size_t packetSize = 0;
	for (size_t p = 0; p < size; p++)
	{
		char currentChar = tel_dataOut[p];
		if(currentChar == '\n')
		{
			if(packetSize + 2 >= TEL_MAX_PACKET_SIZE)
				return 0;
			tel_packetData[packetSize++] = '~';
			tel_packetData[packetSize++] = '0';
		}
		else if(currentChar == '~')
		{
			if(packetSize + 2 >= TEL_MAX_PACKET_SIZE)
				return 0;
			tel_packetData[packetSize++] = '~';
			tel_packetData[packetSize++] = '1';
		}
		else
		{
			if(packetSize + 1 >= TEL_MAX_PACKET_SIZE)
				return 0;
			tel_packetData[packetSize++] = currentChar;
		}
	}
	tel_packetData[packetSize++] = '\n';
	tel_packetSize = packetSize;
	return 1;
}

void tel_sendDataToEsp(void)
{
	if (!tel_espConfigCplt || !tel_sendDataEnabled)
		return;

	uint8_t r, g, b;
	clr_getRgb(&main_robotInstance.colorSensor, &r, &g, &b);
	int cps1 = enc_getCountsPerSecond(&main_robotInstance.rightEncoder);
	int cps2 = enc_getCountsPerSecond(&main_robotInstance.leftEncoder);
	int cnt1 = enc_getCounterValue(&main_robotInstance.rightEncoder);
	int cnt2 = enc_getCounterValue(&main_robotInstance.leftEncoder);

	mpu_Vec3 acc = mpu_readAccData(&main_robotInstance.imu);
	mpu_Vec3 gyro = mpu_readGyroData(&main_robotInstance.imu);
	mpu_Vec3 mag = mpu_readMagData(&main_robotInstance.imu);
	float temp = mpu_readTempData(&main_robotInstance.imu);

	tel_OutputData data = {
		.r = r,
		.g = g,
		.b = b,
		.setPointRightM = main_robotInstance.rightSpeedCtrl.setPoint,
		.setPointLeftM = main_robotInstance.leftSpeedCtrl.setPoint,
		.cpsRightEnc = cps1,
		.cpsLeftEnc = cps2,
		.cntRightEnc = cnt1,
		.cntLeftEnc = cnt2,
		.servoPos = main_robotInstance.servo.position,
#if US_SENSOR
		.distance = main_robotInstance.us.lastDistance,
#elif IR_SENSOR
		.distance = main_robotInstance.ir.lastDistance / 10,
#endif
		.accX = (int16_t)(acc.x * 100),
		.accY = (int16_t)(acc.y * 100),
		.accZ = (int16_t)(acc.z * 100),
		.gyroX = (int16_t)gyro.x,
		.gyroY = (int16_t)gyro.y,
		.gyroZ = (int16_t)gyro.z,
		.magX = (int16_t)(mag.x * 10),
		.magY = (int16_t)(mag.y * 10),
		.magZ = (int16_t)(mag.z * 10),
		.temp = (int16_t)(temp * 10),
		.pitch = (int16_t)(main_robotInstance.orientation.pitch * 10),
		.roll = (int16_t)(main_robotInstance.orientation.roll * 10)
	};

	tel_dataOut[0] = 'D';
	
	memcpy(tel_dataOut + 1, &data, sizeof(tel_OutputData));

	if (!tel_convertDataToPacket(sizeof(tel_OutputData) + 1))
		return;
	
	txt_transmit(&main_robotInstance.espUart, tel_packetData, tel_packetSize);
}

void tel_sendTextToEsp(const char* text)
{
	if (!tel_espConfigCplt)
		return;
	tel_sendDataEnabled = 0;
	tel_dataOut[0] = 'T';

	size_t textLen = strlen(text);
	if (textLen + 1 > TEL_MAX_DATA_SIZE)
		textLen = TEL_MAX_DATA_SIZE - 1;

	memcpy(tel_dataOut + 1, text, textLen);
	if (!tel_convertDataToPacket(textLen + 1))
		return;
	
	txt_transmit(&main_robotInstance.espUart, tel_packetData, tel_packetSize);
	tel_sendDataEnabled = 1;
}
