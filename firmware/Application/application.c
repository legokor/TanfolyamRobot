/*
 * application.c
 */
#include "robotcontrol-api.h"

int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    for(int p = 0; p < 50; p++){
    	uartPrintf("Na csaa\n");
    	uartPrintf("Szia ");
    	uartPrintf("hello!\n");
    }

    delayMs(2000);

    return 0;
}
