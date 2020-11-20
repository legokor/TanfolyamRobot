/*
 * lcd.h
 *
 *  Created on: Aug 31, 2020
 *      Author: ksstms
 */
#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

void lcdInit(GPIO_TypeDef *lcdRstPort, uint16_t lcdRstPin, GPIO_TypeDef *lcdEnPort, uint16_t lcdEnPin,
             GPIO_TypeDef *lcdD4Port, uint16_t lcdD4Pin, GPIO_TypeDef *lcdD5Port, uint16_t lcdD5Pin,
             GPIO_TypeDef *lcdD6Port, uint16_t lcdD6Pin, GPIO_TypeDef *lcdD7Port, uint16_t lcdD7Pin,
             uint8_t lcdRows, uint8_t lcdCols);
int lcdAddCustomCharacter(uint8_t address, const uint8_t character[8]);
int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...);
int lcdPuts(uint8_t row, uint8_t col, const char *str);
int lcdPutc(uint8_t row, uint8_t col, char c);
int lcdSetCursor(uint8_t row, uint8_t col);
void lcdGetCursor(uint8_t* row, uint8_t* col);
int lcdClear();
void lcdHandler();

#endif /* LCD_LCD_H_ */
