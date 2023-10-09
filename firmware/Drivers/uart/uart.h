/*
 * uart.h
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

	uint16_t writeBufferLenght;
	uint16_t readBufferLenght;

	char* writeCircularBuffer;
	int32_t startOfWriteData;
	int32_t endOfWriteData;
	uint8_t transmissionInProgress;

	char* readCircularBuffer;
	uint16_t startOfReadData;
	uint16_t readPtr;
	uint8_t readPtrOverflow;
	int32_t mostRecentNewLinePos;
	uint8_t ok;

	uint8_t readAllChars;
} Uart;


/*
 * Initializes the UART object
 */
void uart_init(volatile Uart* uart, UART_HandleTypeDef *huart, IRQn_Type uartIr, IRQn_Type sendDMAIr, uint8_t readAllChars, uint16_t writeBufferLenght, uint16_t readBufferLenght);

/*
 * Call when the HAL_UART_TxCpltCallback function is called
 */
void uart_handleTransmitCplt(volatile Uart* uart, UART_HandleTypeDef *huart);

/*
 * Call when the HAL_UART_RxCpltCallback function is called
 */
void uart_handleReceiveCplt(volatile Uart* uart, UART_HandleTypeDef *huart);

/*
 * Transmits the data (max lenght is writeBufferLenght)
 */
void uart_transmit(volatile Uart* uart, const char *str);

/*
 * Receives the data until the last received \n, ignores \r (max lenght is readBufferLenght)
 * Puts a \0 at the end of the data, returns false, if no data is available
 */
uint8_t uart_receive(volatile Uart* uart, char* data);



#endif /* UART_UART_H_ */
