/*
 * application.c
 */
#include "robotcontrol-api.h"

const char* WIFI_SSID = "Andi";
const char* WIFI_PASSWORD = "Gomboc02";

#define MANUAL_IP FALSE
const char* SERVER_IP = "192.168.112.82";

int application()
{


    while (1)
    {
#if US_SENSOR
    	uint16_t dist=getUsDistance();
    	lcdPrintf(0, 0, "%03d cm", dist);
#elif IR_SENSOR
    	uint16_t dist=getIrDistance();
    	lcdPrintf(0, 0, "%03d.%01d cm", dist/10, dist%10);
#endif
    	delayMs(200);
    }

    return 0;
}
