/*
 * application.c
 */
#include "robotcontrol-api.h"

int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(2000);

    return 0;
}
