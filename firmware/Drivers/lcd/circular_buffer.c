/*
 * circular_buffer.c
 *
 *  Created on: Oct 14, 2020
 *      Author: ksstms
 */
#include "circular_buffer.h"

/**
 * Increase val by 1, wrap to size
 */
uint16_t inc(uint16_t val, uint16_t size) {
    return (val == size-1) ? (0) : (val + 1);
}

/**
 * Initialize circular buffer struct
 * @param cBuf struct that will store the settings and state of the buffer
 * @param memory to be used to store the data
 * @param size of memory
 */
void circularBufferInit(CircularBuffer* cBuf, uint8_t* memory, uint16_t size) {
    cBuf->memory = memory;
    cBuf->size = size;
    cBuf->elements = 0;
    cBuf->writeIndex = 0;
    cBuf->readIndex = 0;
}

/**
 * Write data to buffer
 * @param cBuf
 * @param data to be written
 * @return 0 on success, -1 when buffer is full
 */
int8_t circularBufferWrite(CircularBuffer* cBuf, uint8_t data) {
    // Can't write, buffer is full
    if (cBuf->elements == cBuf->size) {
        return -1;
    }

    cBuf->memory[cBuf->writeIndex] = data;
    cBuf->elements++;
    cBuf->writeIndex = inc(cBuf->writeIndex, cBuf->size);

    return 0;
}

/**
 * Read data from buffer
 * @param cBuf
 * @param data pointer to store the read data
 * @return 0 on success, -1 when buffer is empty
 */
int8_t circularBufferRead(CircularBuffer* cBuf, uint8_t *data) {
    if (cBuf->elements == 0) {
        return -1;
    }

    *data = cBuf->memory[cBuf->readIndex];
    cBuf->elements--;
    cBuf->readIndex = inc(cBuf->readIndex, cBuf->size);

    return 0;
}
uint8_t circularBufferIsEmpty(CircularBuffer* cBuf) {
    return (cBuf->elements == 0);
}

uint8_t circularBufferIsFull(CircularBuffer* cBuf) {
    return (cBuf->elements == cBuf->size);
}
