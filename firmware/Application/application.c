/*
 * application.c
 */
#include "robotcontrol-api.h"
#include "encoder.h"
#include "main.h"


const uint8_t motorEncoder = MOT_L;

void printEncoderT(Encoder* encoder, uint32_t elapsed){
	if(motorEncoder == MOT_L && encoder == &encoder2)
		uartPrintf("na csa %d\n", elapsed);
	else if(motorEncoder == MOT_R && encoder == &encoder1)
		uartPrintf("na csa %d\n", elapsed);
}


int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(2000);

    return 0;
}
