/*
 * application.c
 */
#include "robotcontrol-api.h"
#include "encoder.h"
#include "main.h"


const uint8_t motorEncoder = MOT_L;
const int stageMs = 2000;
const int stages[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
const int stageNum = 20;

int activeStage = 0;

//elapsed - time since last encoder tick (in 64MHz clock ticks)
void printEncoderT(Encoder* encoder, uint32_t elapsed){
	if((motorEncoder == MOT_L && encoder == &encoder2) || (motorEncoder == MOT_R && encoder == &encoder1))
		uartPrintf("%d;%d\n", activeStage, elapsed);
}


int application() {

    lcdPrintf(0, 0, "Hello");
    lcdPrintf(1, 5, "World!");

    delayMs(1000);

    for(activeStage = 0; activeStage < stageNum; activeStage++){
    	setMotorSpeed(motorEncoder, stages[activeStage]);
    	delayMs(stageMs);
    	setMotorSpeed(motorEncoder, 0);
		delayMs(stageMs);
    }

    return 0;
}
