/*
 * application.c
 */
#include "robotcontrol-api.h"

void printData(){
	Color c;
	uint16_t dist;
	uint32_t encL, encR;

	getColorHsv(&c);
	dist = getUsDistance();
	encL = getEncoderPosition(MOT_L);
	encR = getEncoderPosition(MOT_R);
	lcdPrintf(1, 0, "H%.3dS%.3dV%.3dD%.3d",c.h, c.s, c.v, dist );
	lcdPrintf(0, 0, "R%.6d L%.6d", encR, encL );
}

int application() {
	while(1){
		setServoPosition(0);
		for(int i = 0; i<25; i++){
			setMotorSpeed(MOT_L, 10);
			setMotorSpeed(MOT_R, 10);
			delayMs(200);
			printData();
		}
		setServoPosition(90);
		for(int i = 0; i<25; i++){
			setMotorSpeed(MOT_L, 100);
			setMotorSpeed(MOT_R, 100);
			delayMs(200);
			printData();
		}
		setMotorSpeed(MOT_L, 0);
		setMotorSpeed(MOT_R, 0);
		setServoPosition(0);
		for(int i = 0; i<25; i++){
			delayMs(200);
			printData();
		}
		for(int i = 0; i<25; i++){
			setMotorSpeed(MOT_L, -10);
			setMotorSpeed(MOT_R, -10);
			delayMs(200);
			printData();
		}
		setServoPosition(-90);
		for(int i = 0; i<25; i++){
			setMotorSpeed(MOT_L, -100);
			setMotorSpeed(MOT_R, -100);
			delayMs(200);
			printData();
		}
		setMotorSpeed(MOT_L, 0);
		setMotorSpeed(MOT_R, 0);
		setServoPosition(0);
		for(int i = 0; i<25; i++){
			delayMs(200);
			printData();
		}
	}




    return 0;
}
