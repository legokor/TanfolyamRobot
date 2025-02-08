/*
 * app_main.c
 */
#include "api_robotAbstraction.h"
#include <string.h>
#include <stdio.h>

const char* WIFI_SSID = "1619-LegoDora";
const char* WIFI_PASSWORD = "almaalma";

const char* SERVER_IP = "10.4.3.5";

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
		int power;
		int time;

		if(sscanf(data, "%s %d %d", subCmd, &power, &time) >= 1)
		{
			espPrintf("COMMAND: %s", subCmd);

			if(time > 10000)
				time = 10000;
			if(time < 10)
				time = 10;

			if(power > 60)
				power = 60;
			if(power < 0)
				power = 0;

			if(strcmp(subCmd, "ELORE") == 0)
			{
				setMotorSpeed(MOT_L, power);
				setMotorSpeed(MOT_R, power);
				delayMs(time);
			}
			else if(strcmp(subCmd, "HATRA") == 0)
			{
				setMotorSpeed(MOT_L, -power);
				setMotorSpeed(MOT_R, -power);
				delayMs(time);
			}
			else if(strcmp(subCmd, "JOBBRA") == 0)
			{
				setMotorSpeed(MOT_L, power);
				setMotorSpeed(MOT_R, -power);
				delayMs(time);
			}
			else if(strcmp(subCmd, "BALRA") == 0)
			{
				setMotorSpeed(MOT_L, -power);
				setMotorSpeed(MOT_R, power);
				delayMs(time);
			}
			else if(strcmp(subCmd, "S_JOBBRA") == 0)
			{
				setServoPosition(-90);
			}
			else if(strcmp(subCmd, "S_BALRA") == 0)
			{
				setServoPosition(90);
			}
			else if(strcmp(subCmd, "S_ELORE") == 0)
			{
				setServoPosition(0);
			}
			else if(strcmp(subCmd, "F_JOBBRA") == 0)
			{
				float w = 0;
				setMotorSpeed(MOT_L, power);
				setMotorSpeed(MOT_R, -power);
				while(w >- 90)
				{
					w += getGyroData().z * 0.01f;
					delayUs(10000);
				}
			}
			else if(strcmp(subCmd, "F_BALRA") == 0)
			{
				float w = 0;
				setMotorSpeed(MOT_L, -power);
				setMotorSpeed(MOT_R, power);
				while(w < 90)
				{
					w += getGyroData().z * 0.01f;
					delayUs(10000);
				}
			}
			setMotorSpeed(MOT_L, 0);
			setMotorSpeed(MOT_R, 0);
		}
	}

    return 0;
}
