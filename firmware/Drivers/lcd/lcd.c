/*
 * lcd.c
 *
 *  Created on: Aug 31, 2020
 *      Author: ksstms
 */
#include "lcd.h"
#include <stdarg.h>

/**
 * Initialize the display
 * @param port of the following GPIO pins
 * @param pinRst RST pin number
 * @param pinRw R/W pin number
 * @param pinEn EN pin number
 * @param pinD4 D4 pin number
 * @param pinD5 D5 pin number
 * @param pinD6 D6 pin number
 * @param pinD7 D7 pin number
 * @param timer for timing the bitbanging
 * @param irq of the timer
 * @param numOfLines on LCD
 */
void lcdInit(GPIO_TypeDef *port,
             uint16_t pinRst, uint16_t pinRw, uint16_t pinEn,
             uint16_t pinD4, uint16_t pinD5, uint16_t pinD6, uint16_t pinD7,
             TIM_TypeDef* timer, IRQn_Type irq,
             uint8_t numOfLines) {

    // LINES_2 actually means 2 or more
    hd44780_lines_type lineType = (numOfLines > 1) ? HD44780_LINES_2 : HD44780_LINES_1;

    // Use default font
    hd44780_font_type fontType = HD44780_FONT_5x8;

    // TODO: no clue if this matters
    uint32_t irqSubPriority = 1;

    hd44780_init(port,  pinRst, pinRw, pinEn, pinD4, pinD5, pinD6, pinD7,
                 timer, irq, irqSubPriority, lineType, fontType);
}

/**
 * Works just like printf after the position specifiers
 * @param row of starting position
 * @param col of starting position
 * @param fmt printf-like format string followed by a variable number of arguments
 */
void lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    hd44780_position(row, col);
    hd44780_printf(fmt, args);
}

/**
 * This controls the bitbanging. Call it from the timer's interrupt handler!
 */
void lcdIrqHandler() {
    hd44780_handler();
}
