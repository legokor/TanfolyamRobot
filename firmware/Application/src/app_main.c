/*
 * app_main.c
 */
#include "api_robotAbstraction.h"
#include <string.h>
#include <stdio.h>

const char* WIFI_SSID = "Tanfrobot";
const char* WIFI_PASSWORD = "jelszo1234";

#define MANUAL_IP TRUE
const char* SERVER_IP = "192.168.137.3";

char data[500];
char cmd[100];

int app_main()
{
	lcdPrintf(0, 0, "Gyere LEGO-ba!");
	lcdPrintf(1, 0, "  JOOOLESZ");

	while(1)
	{
		while(!espRead(data)) { }

		char subCmd[100];
		int time;

		if(sscanf(data, "%s %d", subCmd, &time) == 2)
		{
			espPrintf("COMMAND: %s %d", subCmd, time);

			if(time > 10000)
				time = 10000;
			if(time < 10)
				time = 10;

			if(strcmp(subCmd, "ELORE") == 0)
			{
				setMotorSpeed(MOT_L, 60);
				setMotorSpeed(MOT_R, 60);
				delayMs(time);
			}
			else if(strcmp(subCmd, "HATRA") == 0)
			{
				setMotorSpeed(MOT_L, -60);
				setMotorSpeed(MOT_R, -60);
				delayMs(time);
			}
			else if(strcmp(subCmd, "JOBBRA") == 0)
			{
				setMotorSpeed(MOT_L, 60);
				setMotorSpeed(MOT_R, -60);
				delayMs(time);
			}
			else if(strcmp(subCmd, "BALRA") == 0)
			{
				setMotorSpeed(MOT_L, -60);
				setMotorSpeed(MOT_R, 60);
				delayMs(time);
			}
			setMotorSpeed(MOT_L, 0);
			setMotorSpeed(MOT_R, 0);
		}
	}

    return 0;
}
