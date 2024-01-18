/*
 * application.c
 */
#include "robotcontrol-api.h"

const char* WIFI_SSID = "Andi_Vs_ESP_rematch";
const char* WIFI_PASSWORD = "NE FOGD MEG ;)";
const char* SERVER_IP = "192.168.4.1";

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
    	}
    }

    return 0;
}
