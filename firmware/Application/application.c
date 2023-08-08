/*
 * application.c
 */
#include "robotcontrol-api.h"
#include <string.h>

void robotDirection(char * espString, int* dir)
{
	if (strcmp(espString, "FORWARD\n")== 0)
	{
		*dir = 0;
		return;
	}else if (strcmp(espString, "LEFT\n")== 0)
	{
		*dir = 1;
	}else if (strcmp(espString, "BACKWARD\n")== 0)
		{
			*dir = 2;
		}
	else if (strcmp(espString, "RIGHT\n")== 0)
		{
			*dir = 3;
		}
	else if (strcmp(espString, "STOP\n")== 0)
	{
		*dir = 4;
	}

	return;
}



int application() {
	char espData[500];
	int direction = -1;
	int state = 0;
	int ctr = 0;
	lcdPrintf(1,0,"  192.168.4.1  ");
    while(1){
    	if(espRead(espData)){
    		robotDirection(espData, &direction);
    		switch(direction){
				case 0:	// Goes Forward
					{
						setMotorSpeed(MOT_L, 30);
						setMotorSpeed(MOT_R, 30);
						break;
					}
				case 1: // Turns Left
					{
						setMotorSpeed(MOT_L, -30);
						setMotorSpeed(MOT_R, 30);
						break;
					}
				case 2: // Goes Backwards
					{
						setMotorSpeed(MOT_L, -30);
						setMotorSpeed(MOT_R, -30);
						break;
					}
				case 3: // Turns Right
					{
						setMotorSpeed(MOT_L, 30);
						setMotorSpeed(MOT_R, -30);
						break;
					}
				case 4: // Stops Completely
					{
						setMotorSpeed(MOT_L, 0);
						setMotorSpeed(MOT_R, 0);
						delayMs(10);
						break;
					}
				default:
					break;
			}

    		switch(state){
				case 0:
					{
						if(direction==0) state++;
						else state=0;
						break;
					}
				case 1:
					{
						if(direction==2) state++;
						else state=0;
						break;
					}
				case 2:
					{
						if(direction==3) state++;
						else state=0;
						break;
					}
				case 3:
					{
						if(direction==1) {
							lcdPrintf(0,0,"NYOMOD FASZ!!!");
							ctr = 0;
							state=0;
						} else state=0;
						break;
					}
				default:
					break;
    		}

    	}
    	ctr++;
    	if(ctr>3000)
    		lcdPrintf(0,0,"              ");
    	delayMs(10);

    }

    return 0;
}
