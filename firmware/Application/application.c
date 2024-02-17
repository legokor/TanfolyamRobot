/*
 * application.c
 */
#include "robotcontrol-api.h"

const char* WIFI_SSID = "Andi";
const char* WIFI_PASSWORD = "Gomboc02";

#define MANUAL_IP FALSE
const char* SERVER_IP = "192.168.1.239";

int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(2000);

    while(1){
    	char data[300];
    	if(espRead(data)){
    		lcdPrintf(0, 0, "                ");
    		lcdPrintf(1, 0, "                ");
    		lcdPrintf(0, 0, data);
    		espPrintf("I received: %s", data);
    	}
    }

    return 0;
}
