/*
 * uart.c
 *
 *  Created on: Apr 11, 2023
 *      Author: dkiovics
 */

#include "uart.h"


#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>



void uart_init(volatile Uart *uart, UART_HandleTypeDef *huart, IRQn_Type uartIr, IRQn_Type sendDMAIr, uint16_t writeBufferLenght, uint16_t readBufferLenght, char* ignoreableChars){
	uart->huart = huart;
	uart->writeBufferLenght = writeBufferLenght;
	uart->readBufferLenght = readBufferLenght;
	uart->mostRecentNewLinePos = -1;
	uart->startOfReadData = 0;
	uart->readPtr = 0;
	uart->readPtrOverflow = 0;
	uart->uartIr = uartIr;
	uart->sendDMAIr = sendDMAIr;

	uart->startOfWriteData = -1;
	uart->endOfWriteData = writeBufferLenght - 1;
	uart->transmissionInProgress = 0;

	uart->writeCircularBuffer = (char*)malloc(writeBufferLenght+1);
	uart->readCircularBuffer = (char*)malloc(readBufferLenght+1);

	HAL_UART_Receive_IT(uart->huart, (uint8_t*)uart->readCircularBuffer, 1);
	uart->ok = 1;

	uart->ignoreableChars = ignoreableChars;
}


void uart_handleTransmitCplt(volatile Uart *uart, UART_HandleTypeDef *huart){
	if(uart->huart != huart || !uart->ok)
		return;
	if(uart->startOfWriteData == -1){
		uart->transmissionInProgress = 0;
		return;
	}
	int charCount;
	if(uart->startOfWriteData <= uart->endOfWriteData){
		charCount = uart->endOfWriteData - uart->startOfWriteData + 1;
		HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)uart->writeCircularBuffer + uart->startOfWriteData, charCount);
		uart->startOfWriteData = -1;
	}
	else{
		charCount = uart->writeBufferLenght - uart->startOfWriteData;
		HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)uart->writeCircularBuffer + uart->startOfWriteData, charCount);
		uart->startOfWriteData = 0;
	}
}


void uart_transmit(volatile Uart *uart, const char *str){
	int size = strlen(str);

	int spaceTillBufferEnd = uart->writeBufferLenght - uart->endOfWriteData - 1;

	if(spaceTillBufferEnd >= size){
		memcpy((void*)uart->writeCircularBuffer + uart->endOfWriteData + 1, (const void*)str, size);
		HAL_NVIC_DisableIRQ(uart->uartIr);
		HAL_NVIC_DisableIRQ(uart->sendDMAIr);
		if(uart->startOfWriteData == -1){
			if(uart->transmissionInProgress){
				uart->startOfWriteData = uart->endOfWriteData + 1;
			}else{
				HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)uart->writeCircularBuffer + uart->endOfWriteData + 1, size);
				uart->transmissionInProgress = 1;
			}
		}
		uart->endOfWriteData = uart->endOfWriteData + size;
		HAL_NVIC_EnableIRQ(uart->uartIr);
		HAL_NVIC_EnableIRQ(uart->sendDMAIr);
	}else{
		if(spaceTillBufferEnd > 0)
			memcpy((void*)uart->writeCircularBuffer + uart->endOfWriteData + 1, (const void*)str, spaceTillBufferEnd);
		memcpy((void*)uart->writeCircularBuffer, (const void*)str + spaceTillBufferEnd, size - spaceTillBufferEnd);
		HAL_NVIC_DisableIRQ(uart->uartIr);
		HAL_NVIC_DisableIRQ(uart->sendDMAIr);
		if(uart->startOfWriteData == -1){
			if(spaceTillBufferEnd == 0){
				if(uart->transmissionInProgress){
					uart->startOfWriteData = 0;
				}else{
					uart->transmissionInProgress = 1;
					HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)uart->writeCircularBuffer, size);
				}
				uart->endOfWriteData = size - 1;
			}else{
				if(uart->transmissionInProgress){
					uart->startOfWriteData = uart->endOfWriteData + 1;
				}else{
					uart->transmissionInProgress = 1;
					uart->startOfWriteData = 0;
					HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)uart->writeCircularBuffer + uart->endOfWriteData + 1, spaceTillBufferEnd);
				}
				uart->endOfWriteData = size - spaceTillBufferEnd - 1;
			}
		}else{
			uart->endOfWriteData = size - spaceTillBufferEnd - 1;
		}
		HAL_NVIC_EnableIRQ(uart->uartIr);
		HAL_NVIC_EnableIRQ(uart->sendDMAIr);
	}
}

char uart_handleReceiveCplt(volatile Uart *uart, UART_HandleTypeDef *huart, uint8_t initCplt){
	if(uart->huart != huart)
		return 0;

	if(!initCplt){
		HAL_UART_Receive_IT(uart->huart, (uint8_t*)uart->readCircularBuffer, 1);
		return 0;
	}
	char c = uart->readCircularBuffer[uart->readPtr];

	char* ptr = uart->ignoreableChars;
	while(*ptr){
		if(c == *ptr){
			HAL_UART_Receive_IT(uart->huart, (uint8_t*)uart->readCircularBuffer + uart->readPtr, 1);
			return c;
		}
		ptr++;
	}

	if(uart->readCircularBuffer[uart->readPtr] == '\n'){
		uart->mostRecentNewLinePos = uart->readPtr;
	}

	uart->readPtr++;
	if(uart->readPtr == uart->readBufferLenght){
		uart->readPtrOverflow = 1;
		uart->readPtr = 0;
	}

	if(uart->readPtr == uart->mostRecentNewLinePos)
		uart->mostRecentNewLinePos = -1;

	if(uart->readPtr == uart->startOfReadData){
		uart->startOfReadData++;
		if(uart->startOfReadData == uart->readBufferLenght)
			uart->startOfReadData = 0;
	}

	HAL_UART_Receive_IT(uart->huart, (uint8_t*)uart->readCircularBuffer + uart->readPtr, 1);
	return 0;
}


uint8_t uart_receive(volatile Uart *uart, char* data){
	HAL_NVIC_DisableIRQ(uart->uartIr);
	int32_t newLine = uart->mostRecentNewLinePos;
	uint16_t startOfData = uart->startOfReadData;
	if(newLine == -1){
		HAL_NVIC_EnableIRQ(uart->uartIr);
		return 0;
	}
	uart->mostRecentNewLinePos = -1;
	uart->startOfReadData = newLine+1;
	if(uart->startOfReadData == uart->readBufferLenght)
		uart->startOfReadData = 0;
	HAL_NVIC_EnableIRQ(uart->uartIr);

	if(startOfData > newLine){
		uint16_t diff = uart->readBufferLenght - startOfData;
		memcpy(data, (const void*)uart->readCircularBuffer + startOfData, diff);
		memcpy(data + diff, (const void*)uart->readCircularBuffer, newLine + 1);
		data[diff + newLine + 1] = '\0';
	}else{
		memcpy(data, (const void*)uart->readCircularBuffer + startOfData, newLine - startOfData + 1);
		data[newLine - startOfData + 1] = '\0';
	}

	return 1;
}

static volatile char prevData[256] = "";
static volatile uint8_t sendDataEnabled = 1;

#if US_SENSOR
void uart_sendDataToEsp(volatile Uart* espUart, volatile ColorSensor* colorSensor, volatile SpeedControl* speedControl1, volatile SpeedControl* speedControl2, volatile Servo* servo, volatile UltraSonic* us, Mpu9250* imu, Orientation orientation){
#elif IR_SENSOR
void uart_sendDataToEsp(volatile Uart* espUart, volatile ColorSensor* colorSensor, volatile SpeedControl* speedControl1, volatile SpeedControl* speedControl2, volatile Servo* servo, volatile InfraRed* ir, Mpu9250* imu, Orientation orientation){
#else
	#error "No ranging module defined as active"
#endif
	if(!sendDataEnabled)
		return;
	uint8_t r, g, b;
	colorSensorGetRgb(colorSensor, &r, &g, &b);
	int cps1 = encoderGetCountsPerSecond(speedControl1->encoder);
	int cps2 = encoderGetCountsPerSecond(speedControl2->encoder);
	int cnt1 = encoderGetCounterValue(speedControl1->encoder);
	int cnt2 = encoderGetCounterValue(speedControl2->encoder);

	Vec3 acc = mpu9250_readAccData(imu);
	Vec3 gyro = mpu9250_readGyroData(imu);
	Vec3 mag = mpu9250_readMagData(imu);
	float temp = mpu9250_readTempData(imu);

	char data[256];
	//D: R G B Setpoint1 Setpoint2 CPS1 CPS2 CNT1 CNT2 Servo US/IR ACC_X ACC_Y ACC_Z GYRO_X GYRO_Y GYRO_Z MAG_X MAG_Y MAG_Z TEMP
	//For some reason when sprintf is called from an interrupt handler it cannot contain any %f formatted values
	//otherwise it sometimes produces garbage output
	sprintf(prevData, "D: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", (int)r, (int)g, (int)b,
			(int)speedControl1->setPoint, (int)speedControl2->setPoint,
			cps1, cps2, cnt1, cnt2, servo->position, 
#if US_SENSOR
			us->lastDistance,
#elif IR_SENSOR
			ir->lastDistance/10,
#else
	#error "No ranging module defined as active"
#endif
			(int)(acc.x * 100), (int)(acc.y * 100), (int)(acc.z * 100), (int)gyro.x, (int)gyro.y, (int)gyro.z,
			(int)(mag.x * 10), (int)(mag.y * 10), (int)(mag.z * 10), (int)(temp * 10), (int)(orientation.pitch * 10), (int)(orientation.roll * 10));

	strcpy(data, prevData);
	strcat(data, "\n");

	uart_transmit(espUart, data);
}

void uart_sendTextToEsp(volatile Uart* espUart, const char* text){
	char data[512];
	sendDataEnabled = 0;
	sprintf(data, "%s %s\n", prevData, text);
	sendDataEnabled = 1;
	uart_transmit(espUart, data);
}

void uart_sendConfigToEsp(volatile Uart* espUart, const char* SSID, const char* password, const char* IP){
	char data[256];
	sprintf(data, "C:\t%s\t%s\t%s\n", SSID, password, IP);
	uart_transmit(espUart, data);
}

