/*
 * battery_indicator.c
 *
 *  Created on: Nov 15, 2020
 *      Author: ksstms
 */
#include <status_indicator.h>
#include "lcd.h"
#include "../../RobotApi/robotAbstraction-api.h"

#define BATTERY_CELL_COUNT 2

volatile uint8_t enabled = 0;

/**
 * Custom characters for HD44780-based LCD
 */
const uint8_t batteryChar[][8] = {
    {0b01110, 0b10101, 0b11011, 0b01110, 0b01110, 0b10001, 0b01110, 0b10001},   // battery dead
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111},   // battery level 0
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111},   // battery level 1
    {0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111},   // battery level 2
    {0b01110, 0b11011, 0b10001, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 3
    {0b01110, 0b11011, 0b10001, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 4
    {0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},   // battery level 5
};

uint8_t levelNum = sizeof(batteryChar) / 8;

/**
 * Threshold voltages for the battery levels
 */
uint16_t voltageThreshold[] = { BATTERY_CELL_COUNT * 3290,  // 5%
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
void statusIndicatorInit() {
    for (uint8_t i = 0; i < levelNum; i++) {
        lcdAddCustomCharacter(i, &batteryChar[i][0]);
    }
}


void statusIndicatorDisplay(uint8_t row, uint8_t col, uint16_t batteryVoltage, char espState) {
    if (!enabled) {
        return;
    }

    uint8_t c = 0;

    // Find the correct character based on the voltage thresholds
    for (uint8_t i=0; i<levelNum; i++) {

        // At this point we know that the voltage is higher than the highest threshold
        if (i == levelNum - 1) {
            c = i;
            break;
        }

        // If the voltage is <= threshold, then this is the correct character
        if (batteryVoltage <= voltageThreshold[i]) {
          c = i;
          break;
        }
    }

    uint8_t originalRow, originalCol;
    lcdGetCursor(&originalRow, &originalCol);
    lcdPutc(row, col, c);

    switch(espState){
	case 0:
		lcdPutc(1, 15, 'X');	//ESP not detected
		break;
	case 1:
		lcdPutc(1, 15, '~');	//ESP connecting to WiFi
		break;
	case 2:
		lcdPutc(1, 15, '.');	//ESP connecting to server
		break;
	case 3:
		lcdPutc(1, 15, '+');	//ESP connected to server
		break;
	}

    lcdSetCursor(originalRow, originalCol);
}

void statusIndicatorEnable() {
    enabled = 1;
}

void statusIndicatorDisable() {
    enabled = 0;
}
