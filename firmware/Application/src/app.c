/*
 * app.c
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "RobotApi/api_interface.h"
#include "RobotApi/api_misc.h"

const char* WIFI_SSID = "1619-LegoLab";
const char* WIFI_PASSWORD = "almaalma";

const char* SERVER_IP = "10.4.3.5";

int app_main()
{
    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(2000);

    while (1)
    {
        char data[300];
        if (espRead(data))
        {
            espPrintf("I received: %s", data);
        }
    }

    return 0;
}
