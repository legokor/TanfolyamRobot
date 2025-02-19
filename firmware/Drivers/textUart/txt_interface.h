/*
 * txt_interface.h
 *
 *  Created on: Apr 11, 2023
 *      Author: dkiovics
 */

#ifndef UART_UART_H_
#define UART_UART_H_


#include "stm32f1xx_hal.h"


typedef struct {
	UART_HandleTypeDef* huart;
	IRQn_Type uartIr;
	IRQn_Type sendDMAIr;

	volatile uint16_t writeBufferLenght;
	volatile uint16_t readBufferLenght;

	volatile char* writeCircularBuffer;
	volatile int32_t startOfWriteData;
	volatile int32_t endOfWriteData;
	volatile uint8_t transmissionInProgress;

	volatile char* readCircularBuffer;
	volatile uint16_t startOfReadData;
	uint16_t readPtr;
	volatile int32_t mostRecentNewLinePos;
	volatile uint8_t ok;

	const char* ignoreableChars;
} txt_Uart;


/*
 * Initializes the UART object
 */
void txt_init(txt_Uart* uart, UART_HandleTypeDef *huart, IRQn_Type uartIr, IRQn_Type sendDMAIr, uint16_t writeBufferLenght, uint16_t readBufferLenght, const char* ignoreableChars);

/*
 * Call when the HAL_UART_TxCpltCallback function is called
 */
void txt_handleTransmitCplt(txt_Uart* uart, UART_HandleTypeDef *huart);

/*
 * Call when the HAL_UART_RxCpltCallback function is called
 * Returns the received char if it is part of ignoreableChars, otherwise returns 0
 */
char txt_handleReceiveCplt(txt_Uart* uart, UART_HandleTypeDef *huart, uint8_t initCplt);

/*
 * Transmits the data (max lenght is writeBufferLenght)
 */
void txt_transmit(txt_Uart* uart, const char *str, const size_t size);

/*
 * Receives the data until the last received \n, ignores \r (max lenght is readBufferLenght)
 * Puts a \0 at the end of the data, returns false, if no data is available
 */
uint8_t txt_receive(txt_Uart* uart, char* data);

#endif /* UART_UART_H_ */
