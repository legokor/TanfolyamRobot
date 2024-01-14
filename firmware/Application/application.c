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

    return 0;
}
