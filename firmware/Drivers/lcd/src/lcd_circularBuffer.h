/*
 * lcd_circularBuffer.h
 *
 *  Created on: Oct 14, 2020
 *      Author: ksstms
 */
#ifndef _CIRCULAR_BUFFER_H
#define _CIRCULAR_BUFFER_H

#include <stdint.h>

typedef struct {
    uint8_t* memory;
    uint16_t size;
    uint16_t elements;
    uint16_t writeIndex;
    uint16_t readIndex;
} lcd_CircularBuffer;

void lcd_circularBufferInit(lcd_CircularBuffer* cBuf, uint8_t* memory, uint16_t size);

int8_t lcd_circularBufferWrite(lcd_CircularBuffer* cBuf, uint8_t data);

int8_t lcd_circularBufferRead(lcd_CircularBuffer* cBuf, uint8_t *data);

uint16_t lcd_circularBufferGetAvailable(lcd_CircularBuffer* cBuf);

uint8_t lcd_circularBufferIsEmpty(lcd_CircularBuffer* cBuf);

uint8_t lcd_circularBufferIsFull(lcd_CircularBuffer* cBuf);

#endif //_CIRCULAR_BUFFER_H
