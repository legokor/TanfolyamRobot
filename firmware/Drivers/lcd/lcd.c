/*
 * lcd.c
 *
 *  Created on: Aug 31, 2020
 *      Author: ksstms
 *
 * Most public functions of this library are non-blocking. They just put commands
 * in a buffer, which are sent via the handler function. The handler function shall
 * be called from a timer interrupt. Calls should occur at least ~20 us apart, but
 * the exact minimum might be display dependent.
 *
 * This library is largely based on Nima Askari's work found at:
 * https://github.com/nimaltd/LCD-Character
 */
#include "lcd.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "circular_buffer.h"

/*
 * This many transfers can be waiting for transmission at once
 */
#define TRANSFER_BUFFER_SIZE 256

/*
 * HD44780-related stuff
 */
// Commands
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80
// Flags for display entry mode
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00
// Flags for display on/off control
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01
// Flags for display/cursor shift
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00
// Flags for function set
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

/*
 * HD44780 character addressing works like this (if screen width <= 20):
 * | 0x00    1st row    0x13 | 0x14    2nd row    0x39 |
 * | 0x00    2nd row    0x13 | 0x14    2nd row    0x39 |
 */
const uint8_t HD44780RowOffsets[] = {0x00, 0x40, 0x14, 0x54};

/*
 * Store settings and state of the display
 */
typedef struct {
    uint8_t displayControl;
    uint8_t displayFunction;
    uint8_t displayEntryMode;
    uint8_t numOfRows;
    uint8_t numOfCols;
} LcdSettings;

typedef struct {
    uint8_t currentCol;
    uint8_t currentRow;
    uint8_t initialized;
} LcdState;

static LcdSettings lcdSettings;
static LcdState lcdState;

/*
 * Transfer buffer for storing commands to be sent
 */
typedef enum {
    TransferType_Command = 0,
    TransferType_Data = 1,
} TransferType;

CircularBuffer transferBuf;
uint8_t transferBufMem[TRANSFER_BUFFER_SIZE];
uint8_t enPinState = 0;

/*
 * LCD pin locations
 */
GPIO_TypeDef *rsPort;
GPIO_TypeDef *enPort;
GPIO_TypeDef *d4Port;
GPIO_TypeDef *d5Port;
GPIO_TypeDef *d6Port;
GPIO_TypeDef *d7Port;
uint16_t rsPin;
uint16_t enPin;
uint16_t d4Pin;
uint16_t d5Pin;
uint16_t d6Pin;
uint16_t d7Pin;

/*
 * Private functions
 */
static void lcdCmd4Bit(uint8_t cmd);
static void lcdCmd(uint8_t cmd);
static void lcdDelayMs(uint8_t ms);
static int8_t lcdAdd4BitTransfer(uint8_t val, TransferType type);
static int8_t lcdAddCmd(uint8_t cmd);
static int8_t lcdAddData(uint8_t data);
static int8_t lcdSetCursor(uint8_t row, uint8_t col);
/**
 * Initialize the display
 * @param lcdXPort GPIO port of X
 * @param lcdXPin GPIO pin number of X
 */
void lcdInit(GPIO_TypeDef *lcdRsPort, uint16_t lcdRsPin, GPIO_TypeDef *lcdEnPort, uint16_t lcdEnPin,
             GPIO_TypeDef *lcdD4Port, uint16_t lcdD4Pin, GPIO_TypeDef *lcdD5Port, uint16_t lcdD5Pin,
             GPIO_TypeDef *lcdD6Port, uint16_t lcdD6Pin, GPIO_TypeDef *lcdD7Port, uint16_t lcdD7Pin,
             uint8_t lcdRows, uint8_t lcdCols)
{
    lcdState.initialized = 0;

    // Save ports and pins
    rsPort = lcdRsPort;
    enPort = lcdEnPort;
    d4Port = lcdD4Port;
    d5Port = lcdD5Port;
    d6Port = lcdD6Port;
    d7Port = lcdD7Port;
    rsPin = lcdRsPin;
    enPin = lcdEnPin;
    d4Pin = lcdD4Pin;
    d5Pin = lcdD5Pin;
    d6Pin = lcdD6Pin;
    d7Pin = lcdD7Pin;

    // Initialize GPIOs
    GPIO_InitTypeDef  gpio;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = enPin;
    HAL_GPIO_Init(enPort, &gpio);
    HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_RESET);
    enPinState = 0;

    gpio.Pin = rsPin;
    HAL_GPIO_Init(rsPort, &gpio);
    gpio.Pin = d4Pin;
    HAL_GPIO_Init(d4Port, &gpio);
    gpio.Pin = d5Pin;
    HAL_GPIO_Init(d5Port, &gpio);
    gpio.Pin = d6Pin;
    HAL_GPIO_Init(d6Port, &gpio);
    gpio.Pin = d7Pin;
    HAL_GPIO_Init(d7Port, &gpio);

    // Initialize transfer buffer
    circularBufferInit(&transferBuf, transferBufMem, TRANSFER_BUFFER_SIZE);

    lcdDelayMs(1);
    lcdState.currentCol = 0;
    lcdState.currentRow = 0;
    lcdSettings.numOfRows = lcdRows;
    lcdSettings.numOfCols = lcdCols;
    lcdSettings.displayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_2LINE;
    lcdSettings.displayControl = HD44780_DISPLAYON;
    lcdSettings.displayEntryMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;

    // Try to set 4bit mode
    for (uint8_t i=0; i<3; i++) {
        lcdCmd4Bit(0x03);
        lcdDelayMs(5);
    }
    // Set 4-bit interface
    lcdCmd4Bit(0x02);
    lcdDelayMs(5);

    // Set functions
    lcdCmd(HD44780_FUNCTIONSET | lcdSettings.displayFunction);

    // Turn on display
    lcdCmd(HD44780_DISPLAYCONTROL | lcdSettings.displayControl);
    lcdCmd(HD44780_CLEARDISPLAY);
    lcdDelayMs(5);

    // Set
    lcdCmd(HD44780_ENTRYMODESET | lcdSettings.displayEntryMode);
    lcdDelayMs(5);

    lcdSetCursor(0, 0);
    lcdState.initialized = 1;
}


/**
 * Works just like printf after the position specifiers
 * @param row of starting position
 * @param col of starting position
 * @param fmt printf-like format string followed by a variable number of arguments
 */
int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char str[64];
    int size = vsprintf(str, fmt, args);

    if (size <= 0) {
        return -1;
    }

    return lcdPuts(row, col, str);
}

/**
 * Put string on the LCD
 * @param row of starting position
 * @param col of starting position
 * @param str string to be displayed
 */
int lcdPuts(uint8_t row, uint8_t col, const char *str) {
    int err;

    err = lcdSetCursor(row, col);
    if (err) {
        return err;
    }

    uint8_t printedChars = 0;

    while (*str) {
        if (lcdState.currentCol >= lcdSettings.numOfCols) {
            lcdState.currentCol = 0;
            lcdState.currentRow++;
            err = lcdSetCursor(lcdState.currentRow, lcdState.currentCol);
            if (err) {
                return err;
            }
        }

        if (*str == '\n') {
           lcdState.currentRow++;
           lcdState.currentCol=0;
           err = lcdSetCursor(lcdState.currentRow, lcdState.currentCol);
           if (err) {
               return err;
           }

        } else if (*str == '\r') {
            lcdState.currentCol = 0;
            err = lcdSetCursor(lcdState.currentRow, lcdState.currentCol);
            if (err) {
               return err;
            }

        } else {
            lcdAddData(*str);
            lcdState.currentCol++;
        }

        printedChars++;
        str++;
    }

    return printedChars;
}

/**
 * Clear the screen
 * This function is blocking for 5 ms so no other commands will be sent until the screen is cleared
 * @return 0 on success
 */
int lcdClear() {
    int err = lcdAddCmd(HD44780_CLEARDISPLAY);
    if (err) {
        return err;
    }
    lcdDelayMs(5);
}

/**
 * This controls the bitbanging. Call it from a timer's interrupt handler!
 */
void lcdHandler() {
    if (!lcdState.initialized) {
        return;
    }

    if (enPinState == 1) {
        HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_RESET);
        enPinState = 0;
    } else {
        uint8_t cmd;
        int err = circularBufferRead(&transferBuf, &cmd);

        if (!err) {
            HAL_GPIO_WritePin(rsPort, rsPin, (GPIO_PinState)(cmd & (1 << 7)));
            HAL_GPIO_WritePin(d7Port, d7Pin, (GPIO_PinState)(cmd & (1 << 3)));
            HAL_GPIO_WritePin(d6Port, d6Pin, (GPIO_PinState)(cmd & (1 << 2)));
            HAL_GPIO_WritePin(d5Port, d5Pin, (GPIO_PinState)(cmd & (1 << 1)));
            HAL_GPIO_WritePin(d4Port, d4Pin, (GPIO_PinState)(cmd & (1 << 0)));
            HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_SET);
            enPinState = 1;
        }
    }
}

/**
 * Blocking function to send a 4-bit command
 * @param cmd 4-bit command
 */
static void lcdCmd4Bit(uint8_t cmd) {
    HAL_GPIO_WritePin(rsPort, rsPin, (GPIO_PinState)(cmd & (TransferType_Command << 7)));

    HAL_GPIO_WritePin(d7Port, d7Pin, (GPIO_PinState)(cmd & (1 << 3)));
    HAL_GPIO_WritePin(d6Port, d6Pin, (GPIO_PinState)(cmd & (1 << 2)));
    HAL_GPIO_WritePin(d5Port, d5Pin, (GPIO_PinState)(cmd & (1 << 1)));
    HAL_GPIO_WritePin(d4Port, d4Pin, (GPIO_PinState)(cmd & (1 << 0)));

    HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_SET);
    lcdDelayMs(1);
    HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_RESET);
    lcdDelayMs(1);
}

/**
 * Blocking function to send an 8-bit command
 * @param cmd 8-bit command
 */
static void lcdCmd(uint8_t cmd) {
    lcdCmd4Bit(cmd >> 4);
    lcdCmd4Bit(cmd & 0x0F);
}

/**
 * Blocking delay
 * @param ms to wait
 */
static void lcdDelayMs(uint8_t ms) {
    HAL_Delay(ms);
}

/**
 * Add a command or data transfer to the buffer
 * @param val 4-bit part of command or data
 * @param type command or data
 * @return 0 on success
 */
static int8_t lcdAdd4BitTransfer(uint8_t val, TransferType type) {
    val &= 0x0F;
    val = val | (type << 7);

    return circularBufferWrite(&transferBuf, val);
}

/**
 * Add an 8-bit command to the buffer
 * @param cmd 8-bit command
 * @return 0 on success
 */
static int8_t lcdAddCmd(uint8_t cmd) {
    uint8_t cmd0 = cmd >> 4;
    uint8_t cmd1 = cmd & 0x0F;

    int8_t err;
    err = lcdAdd4BitTransfer(cmd0, TransferType_Command);
    if (err) {
        return err;
    }
    err = lcdAdd4BitTransfer(cmd1, TransferType_Command);
    if (err) {
        return err;
    }
    return 0;
}

/**
 * Add an 8-bit data to the buffer
 * @param data 8-bit data
 * @return 0 on success
 */
static int8_t lcdAddData(uint8_t data) {
    uint8_t data0 = data >> 4;
    uint8_t data1 = data & 0x0F;

    int8_t err;
    err = lcdAdd4BitTransfer(data0, TransferType_Data);
    if (err) {
        return err;
    }
    err = lcdAdd4BitTransfer(data1, TransferType_Data);
    if (err) {
        return err;
    }
    return 0;
}

/**
 * Set the LCD's cursor position
 * @param col 0-based column number
 * @param row 0-based row number
 * @return 0 on success
 */
static int8_t lcdSetCursor(uint8_t row, uint8_t col) {
    if (row >= lcdSettings.numOfRows) {
        row = 0;
    }
    lcdState.currentCol = col;
    lcdState.currentRow = row;

    return lcdAddCmd(HD44780_SETDDRAMADDR | (col + HD44780RowOffsets[row]));
}

