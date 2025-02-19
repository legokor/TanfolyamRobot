/*
 * lcd_interface.h
 *
 *  Created on: Feb 1, 2025
 *      Author: dkiovics
 */

#ifndef LCD_LCD_INTERFACE_H_
#define LCD_LCD_INTERFACE_H_

#include "stm32f1xx_hal.h"

void lcd_init(GPIO_TypeDef *lcdRstPort, uint16_t lcdRstPin, GPIO_TypeDef *lcdEnPort, uint16_t lcdEnPin,
             GPIO_TypeDef *lcdD4Port, uint16_t lcdD4Pin, GPIO_TypeDef *lcdD5Port, uint16_t lcdD5Pin,
             GPIO_TypeDef *lcdD6Port, uint16_t lcdD6Pin, GPIO_TypeDef *lcdD7Port, uint16_t lcdD7Pin,
             uint8_t lcdRows, uint8_t lcdCols);

int lcd_puts(uint8_t row, uint8_t col, const char *str);

int lcd_clear();

void lcd_handler();

void lcd_displayStatus(uint8_t row, uint8_t col, uint16_t batteryVoltage, char espState);

void lcd_enableStatus();

void lcd_disableStatus();

#endif /* LCD_LCD_INTERFACE_H_ */
