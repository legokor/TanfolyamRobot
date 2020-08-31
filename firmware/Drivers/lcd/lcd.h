/*
 * lcd.h
 *
 *  Created on: Aug 31, 2020
 *      Author: ksstms
 */
#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "hd44780.h"

void lcdInit(GPIO_TypeDef *port,
             uint16_t pinRs, uint16_t pinRw, uint16_t pinE,
             uint16_t pinD4, uint16_t pinD5, uint16_t pinD6, uint16_t pinD7,
             TIM_TypeDef* timer, IRQn_Type irq,
             uint8_t numOfLines);
void lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...);
void lcdIrqHandler();

#endif /* LCD_LCD_H_ */
