/*
 * lcd_lcd.h
 *
 *  Created on: Aug 31, 2020
 *      Author: ksstms
 */
#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include <stdint.h>

int lcd_addCustomCharacter(uint8_t address, const uint8_t character[8]);

int lcd_putc(uint8_t row, uint8_t col, char c);

int lcd_setCursor(uint8_t row, uint8_t col);

void lcd_getCursor(uint8_t* row, uint8_t* col);

#endif /* LCD_LCD_H_ */
