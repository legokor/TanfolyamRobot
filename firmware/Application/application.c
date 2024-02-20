/*
 * application.c
 */
#include "robotcontrol-api.h"

const char* WIFI_SSID = "Tanfolyam_WiFi";
const char* WIFI_PASSWORD = "LegoFTW1";

#define MANUAL_IP FALSE
const char* SERVER_IP = "192.168.186.102";

int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(2000);

    while(1){
    	char data[300];
    	if(espRead(data)){
    		espPrintf("I received: %s", data);
    	}
    }

    return 0;
}
