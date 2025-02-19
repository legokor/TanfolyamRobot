/*
 * app_main.c
 */
#include "api_robotAbstraction.h"
#include <string.h>
#include <stdio.h>

const char* WIFI_SSID = "LegoTanfolyam";
const char* WIFI_PASSWORD = "almaalma";

const char* SERVER_IP = "192.168.1.69";

int app_main()
{
	lcdPrintf(0, 0, "Hello");
	lcdPrintf(1, 5, "World!");

	delayMs(2000);

	while(1)
	{
		char data[300];
		if (espRead(data))
		{
			espPrintf("I received: %s", data);
		}
	}

	return 0;
}
