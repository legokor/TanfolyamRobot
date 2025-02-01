/*
 * lcd_statusIndicator.c
 *
 *  Created on: Nov 15, 2020
 *      Author: ksstms, dkiovics
 */
#include "lcd_interface.h"
#include "lcd_lcd.h"

#define BATTERY_CELL_COUNT 2

static volatile uint8_t lcd_statusEnabled = 0;

/**
 * Custom characters for HD44780-based LCD
 */
static const uint8_t lcd_batteryChar[][8] = {
    {0b01110, 0b10101, 0b11011, 0b01110, 0b01110, 0b10001, 0b01110, 0b10001},   // battery dead
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111},   // battery level 0
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111},   // battery level 1
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111},   // battery level 2
    {0b01110, 0b11011, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 3
    {0b01110, 0b11011, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 4
    {0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 5
};

static const uint8_t lcd_levelNum = sizeof(lcd_batteryChar) / 8;

/**
 * Threshold voltages for the battery levels
 */
static const uint16_t lcd_voltageThreshold[] = { BATTERY_CELL_COUNT * 3290,  // 5%
                                BATTERY_CELL_COUNT * 3380,  // 10%
                                BATTERY_CELL_COUNT * 3580,  // 30%
                                BATTERY_CELL_COUNT * 3720,  // 50%
                                BATTERY_CELL_COUNT * 3870,  // 70%
                                BATTERY_CELL_COUNT * 4060,  // 90%
                                BATTERY_CELL_COUNT * 4200   // 100%
};

/**
 * Write battery indicator characters to the LCD's CGRAM
 */
void lcd_statusIndicatorInit() {
    for (uint8_t i = 0; i < lcd_levelNum; i++) {
        lcd_addCustomCharacter(i, &lcd_batteryChar[i][0]);
    }
}


void lcd_displayStatus(uint8_t row, uint8_t col, uint16_t batteryVoltage, char espState) {
    if (!lcd_statusEnabled) {
        return;
    }

    uint8_t c = 0;

    // Find the correct character based on the voltage thresholds
    for (uint8_t i=0; i<lcd_levelNum; i++) {

        // At this point we know that the voltage is higher than the highest threshold
        if (i == lcd_levelNum - 1) {
            c = i;
            break;
        }

        // If the voltage is <= threshold, then this is the correct character
        if (batteryVoltage <= lcd_voltageThreshold[i]) {
          c = i;
          break;
        }
    }

    uint8_t originalRow, originalCol;
    lcd_getCursor(&originalRow, &originalCol);
    lcd_putc(row, col, c);

    switch(espState){
	case 0:
		lcd_putc(1, 15, 'X');	//ESP not detected
		break;
	case 1:
		lcd_putc(1, 15, '~');	//ESP connecting to WiFi
		break;
	case 2:
		lcd_putc(1, 15, '.');	//ESP connecting to server
		break;
	case 3:
		lcd_putc(1, 15, '+');	//ESP connected to server
		break;
	}

    lcd_setCursor(originalRow, originalCol);
}

void lcd_enableStatus() {
    lcd_statusEnabled = 1;
}

void lcd_disableStatus() {
    lcd_statusEnabled = 0;
}
