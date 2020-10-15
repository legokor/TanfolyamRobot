/*
 * circular_buffer.h
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
} CircularBuffer;

void circularBufferInit(CircularBuffer* cBuf, uint8_t* memory, uint16_t size);
int8_t circularBufferWrite(CircularBuffer* cBuf, uint8_t data);
int8_t circularBufferRead(CircularBuffer* cBuf, uint8_t *data);
uint8_t circularBufferIsEmpty(CircularBuffer* cBuf);
uint8_t circularBufferIsFull(CircularBuffer* cBuf);

#endif //_CIRCULAR_BUFFER_H
