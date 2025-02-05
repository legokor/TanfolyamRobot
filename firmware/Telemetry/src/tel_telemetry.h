/*
 * tel_telemetry.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* SERVER_IP;

#include <stdint.h>

struct __attribute__((__packed__)) tel_OutputData
{
	uint8_t r, g, b;
	int8_t setPointRightM, setPointLeftM;
	int16_t cpsRightEnc, cpsLeftEnc;
	int32_t cntRightEnc, cntLeftEnc;
	int8_t servoPos;
	uint8_t distance;
	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;
	int16_t magX, magY, magZ;
	int16_t temp;
	int16_t pitch, roll;
};

typedef struct tel_OutputData tel_OutputData;

uint8_t tel_convertDataToPacket(size_t size);

#endif /* TELEMETRY_H_ */
