/*
 * lcd_circularBuffer.c
 *
 *  Created on: Oct 14, 2020
 *      Author: ksstms
 */
#include "lcd_circularBuffer.h"

/**
 * Increase val by 1, wrap to size
 */
static uint16_t lcd_inc(uint16_t val, uint16_t size) {
    return (val == size-1) ? (0) : (val + 1);
}

/**
 * Initialize circular buffer struct
 * @param cBuf struct that will store the settings and state of the buffer
 * @param memory to be used to store the data
 * @param size of memory
 */
void lcd_circularBufferInit(lcd_CircularBuffer* cBuf, uint8_t* memory, uint16_t size) {
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
int8_t lcd_circularBufferWrite(lcd_CircularBuffer* cBuf, uint8_t data) {
    // Can't write, buffer is full
    if (cBuf->elements == cBuf->size) {
        return -1;
    }

    cBuf->memory[cBuf->writeIndex] = data;
    cBuf->elements++;
    cBuf->writeIndex = lcd_inc(cBuf->writeIndex, cBuf->size);

    return 0;
}

/**
 * Read data from buffer
 * @param cBuf
 * @param data pointer to store the read data
 * @return 0 on success, -1 when buffer is empty
 */
int8_t lcd_circularBufferRead(lcd_CircularBuffer* cBuf, uint8_t *data) {
    if (cBuf->elements == 0) {
        return -1;
    }

    *data = cBuf->memory[cBuf->readIndex];
    cBuf->elements--;
    cBuf->readIndex = lcd_inc(cBuf->readIndex, cBuf->size);

    return 0;
}

uint16_t lcd_circularBufferGetAvailable(lcd_CircularBuffer* cBuf) {
    return cBuf->size - cBuf->elements;
}

uint8_t lcd_circularBufferIsEmpty(lcd_CircularBuffer* cBuf) {
    return (cBuf->elements == 0);
}

uint8_t lcd_circularBufferIsFull(lcd_CircularBuffer* cBuf) {
    return (cBuf->elements == cBuf->size);
}
