/*
 * txt_uart.c
 *
 *  Created on: Apr 11, 2023
 *      Author: dkiovics
 */

#include "txt_interface.h"


#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>



void txt_init(txt_Uart *uart, UART_HandleTypeDef *huart, IRQn_Type uartIr, IRQn_Type sendDMAIr, uint16_t writeBufferLenght, uint16_t readBufferLenght, const char* ignoreableChars){
	uart->huart = huart;
	uart->writeBufferLenght = writeBufferLenght;
	uart->readBufferLenght = readBufferLenght;
	uart->mostRecentNewLinePos = -1;
	uart->startOfReadData = 0;
	uart->readPtr = 0;
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


void txt_handleTransmitCplt(txt_Uart *uart, UART_HandleTypeDef *huart){
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

void txt_transmit(txt_Uart *uart, const char *str, const size_t size){
	if(size > uart->writeBufferLenght || size == 0)
		return;

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

char txt_handleReceiveCplt(txt_Uart *uart, UART_HandleTypeDef *huart, uint8_t initCplt){
	if(uart->huart != huart)
		return 0;

	if(!initCplt){
		HAL_UART_Receive_IT(uart->huart, (uint8_t*)uart->readCircularBuffer, 1);
		return 0;
	}
	char c = uart->readCircularBuffer[uart->readPtr];

	const char* ptr = uart->ignoreableChars;
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


uint8_t txt_receive(txt_Uart *uart, char* data){
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
		memcpy(data + diff, (const void*)uart->readCircularBuffer, newLine);
		data[diff + newLine] = '\0';
	}else{
		memcpy(data, (const void*)uart->readCircularBuffer + startOfData, newLine - startOfData);
		data[newLine - startOfData] = '\0';
	}

	return 1;
}

