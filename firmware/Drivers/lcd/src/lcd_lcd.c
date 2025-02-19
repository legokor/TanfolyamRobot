/*
 * lcd_lcd.c
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
#include "lcd_lcd.h"
#include "lcd_interface.h"
#include "lcd_circularBuffer.h"

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
 * | 0x00     row 1     0x13 | 0x14     row 3     0x27 |
 * | 0x40     row 2     0x53 | 0x54     row 4     0x67 |
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
    volatile uint8_t initialized;
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

lcd_CircularBuffer transferBuf;
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
static void lcdSend4Bit(uint8_t data, TransferType type);
static void lcdCmd(uint8_t cmd);
static void lcdDelayMs(uint8_t ms);
static int lcdAdd4BitTransfer(uint8_t val, TransferType type);
static int lcdAddCmd(uint8_t cmd);
static int lcdAddData(uint8_t data);

void lcd_statusIndicatorInit();

/**
 * Initialize the display
 * @param lcdXPort GPIO port of X
 * @param lcdXPin GPIO pin number of X
 */
void lcd_init(GPIO_TypeDef *lcdRsPort, uint16_t lcdRsPin, GPIO_TypeDef *lcdEnPort, uint16_t lcdEnPin,
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
    lcd_circularBufferInit(&transferBuf, transferBufMem, TRANSFER_BUFFER_SIZE);

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
        lcdSend4Bit(0x03, TransferType_Command);
        lcdDelayMs(5);
    }
    // Set 4-bit interface
    lcdSend4Bit(0x02, TransferType_Command);
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

    lcd_setCursor(0, 0);
    lcdState.initialized = 1;

    lcd_statusIndicatorInit();
}

/**
 * Write a custom character in the LCD's CGRAM
 * @note CGRAM is located at character address 0-7, and they are duplicated at 8-15
 * @param address index of the character 0-7
 * @param character 8 rows of 5 bits
 * @return 0 on success
 */
int lcd_addCustomCharacter(uint8_t address, const uint8_t character[8]) {
    if (address >= 8) {
        return -1;
    }

    int err;

    address *= 8;
    err = lcdAddCmd(HD44780_SETCGRAMADDR | address);
    if (err) {
        return err;
    }

    for (uint8_t i=0; i<8; i++) {
        err = lcdAddData(character[i]);
        if (err) {
            return err;
        }
    }

    err = lcd_setCursor(lcdState.currentRow, lcdState.currentCol);

    return err;
}

/**
 * Put string on the LCD
 * @note \n and \r are the only special characters supported. Everything else will be sent to the LCD as-is.
 *
 * @param row of starting position
 * @param col of starting position
 * @param str string to be displayed
 */
int lcd_puts(uint8_t row, uint8_t col, const char *str) {
    int err;

    if ( (lcdState.currentCol != col) || (lcdState.currentRow != row) ) {
        err = lcd_setCursor(row, col);
        if (err) {
            return err;
        }
    }

    uint8_t printedChars = 0;

    while (*str) {

        // And the end of line, jump to the next one
        if (lcdState.currentCol >= lcdSettings.numOfCols) {
            err = lcd_setCursor(lcdState.currentRow + 1, 0);
            if (err) {
                return err;
            }
        }

        if (*str == '\n') {             // Jump to the beginning of the next line
           err = lcd_setCursor(lcdState.currentRow + 1, 0);
           if (err) {
               return err;
           }

        } else if (*str == '\r') {      // Jump to the beginning of the current line
            err = lcd_setCursor(lcdState.currentRow, 0);
            if (err) {
               return err;
            }

        } else {
            err = lcdAddData(*str);
            if (err) {
               return err;
            }
            lcdState.currentCol++;
        }

        printedChars++;
        str++;
    }

    return printedChars;
}

/**
 * Put a single character on the LCD
 * @param row of position
 * @param col of position
 * @param c character to be displayed
 */
int lcd_putc(uint8_t row, uint8_t col, char c) {
    if ( (lcdState.currentCol != col) || (lcdState.currentRow != row) ) {
        int err = lcd_setCursor(row, col);
        if (err) {
            return err;
        }
    }

    int err = lcdAddData(c);
    if (err) {
        return err;
    }
    lcdState.currentCol++;

    return 0;
}

/**
 * Clear the screen
 * This function is blocking for 5 ms so no other commands will be sent until the screen is cleared
 * @return 0 on success
 */
int lcd_clear() {
    int err = lcdAddCmd(HD44780_CLEARDISPLAY);
    if (err) {
        return err;
    }
    lcdDelayMs(5);

    lcdState.currentCol = 0;
    lcdState.currentRow = 0;

    return 0;
}

/**
 * This controls the bitbanging. Call it from a timer's interrupt handler!
 */
void lcd_handler() {
    if (!lcdState.initialized) {
        return;
    }

    if (enPinState == 1) {
        HAL_GPIO_WritePin(enPort, enPin, GPIO_PIN_RESET);
        enPinState = 0;
    } else {
        uint8_t cmd;
        __disable_irq();
        int err = lcd_circularBufferRead(&transferBuf, &cmd);
        __enable_irq();

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
static void lcdSend4Bit(uint8_t data, TransferType type) {
    HAL_GPIO_WritePin(rsPort, rsPin, type);

    HAL_GPIO_WritePin(d7Port, d7Pin, (GPIO_PinState)(data & (1 << 3)));
    HAL_GPIO_WritePin(d6Port, d6Pin, (GPIO_PinState)(data & (1 << 2)));
    HAL_GPIO_WritePin(d5Port, d5Pin, (GPIO_PinState)(data & (1 << 1)));
    HAL_GPIO_WritePin(d4Port, d4Pin, (GPIO_PinState)(data & (1 << 0)));

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
    lcdSend4Bit(cmd >> 4,   TransferType_Command);
    lcdSend4Bit(cmd & 0x0F, TransferType_Command);
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
static int lcdAdd4BitTransfer(uint8_t val, TransferType type) {
    val &= 0x0F;
    val = val | (type << 7);

    int8_t e = lcd_circularBufferWrite(&transferBuf, val);

    return e;
}

/**
 * Add an 8-bit command to the buffer
 * @param cmd 8-bit command
 * @return 0 on success
 */
static int lcdAddCmd(uint8_t cmd) {
    // cmd is sent in two 4-bit parts, so check if those will fit in the buffer
    if (lcd_circularBufferGetAvailable(&transferBuf) < 2) {
        return -1;
    }

    uint8_t cmd0 = cmd >> 4;
    uint8_t cmd1 = cmd & 0x0F;

    __disable_irq();
    int err;
    err = lcdAdd4BitTransfer(cmd0, TransferType_Command);     // this should never fail
    if (!err) {
        err = lcdAdd4BitTransfer(cmd1, TransferType_Command); // this should never fail
    }
    __enable_irq();

    return err;
}

/**
 * Add an 8-bit data to the buffer
 * @param data 8-bit data
 * @return 0 on success
 */
static int lcdAddData(uint8_t data) {
    // data is sent in two 4-bit parts, so check if those will fit in the buffer
    if (lcd_circularBufferGetAvailable(&transferBuf) < 2) {
        return -1;
    }

    uint8_t data0 = data >> 4;
    uint8_t data1 = data & 0x0F;

    __disable_irq();
    int err;
    err = lcdAdd4BitTransfer(data0, TransferType_Data);     // this should never fail
    if (!err) {
        err = lcdAdd4BitTransfer(data1, TransferType_Data); // this should never fail
    }
    __enable_irq();

    return err;
}

/**
 * Set the LCD's cursor position
 * @param col 0-based column number
 * @param row 0-based row number
 * @return 0 on success
 */
int lcd_setCursor(uint8_t row, uint8_t col) {
    if (row >= lcdSettings.numOfRows) {
        row = 0;
    }

    int err = lcdAddCmd(HD44780_SETDDRAMADDR | (col + HD44780RowOffsets[row]));
    if (err) {
        return err;
    }

    lcdState.currentCol = col;
    lcdState.currentRow = row;

    return 0;
}

/**
 * Check the current position of the cursor
 * @param row
 * @param col
 */
void lcd_getCursor(uint8_t* row, uint8_t* col) {
    *row = lcdState.currentRow;
    *col = lcdState.currentCol;
}
