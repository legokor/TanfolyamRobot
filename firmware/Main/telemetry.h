/*
 * telemetry.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "robotInstance.h"

typedef struct
{
	volatile char prevData[256];
	volatile uint8_t sendDataEnabled;
} main_Telemetry;


void main_telemetryInit(main_Telemetry* telemetry);

void main_sendDataToEsp(main_Telemetry* telemetry, main_RobotInstance* robotInstance);

void main_sendTextToEsp(main_Telemetry* telemetry, main_RobotInstance* robotInstance, const char* text);

void main_sendConfigToEsp(main_RobotInstance* robotInstance);


#endif /* TELEMETRY_H_ */
