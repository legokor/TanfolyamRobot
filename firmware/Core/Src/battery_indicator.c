/*
 * battery_indicator.c
 *
 *  Created on: Nov 15, 2020
 *      Author: ksstms
 */
#include "battery_indicator.h"
#include "lcd.h"
#include "robotcontrol-api.h"

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
void batteryIndicatorInit() {
    for (uint8_t i = 0; i < levelNum; i++) {
        lcdAddCustomCharacter(i, &batteryChar[i][0]);
    }
}

/**
 * Display battery indicator
 * @param row of the character position
 * @param col of the character position
 * @param batteryVoltage in mV
 */
void batteryIndicatorDisplay(uint8_t row, uint8_t col, uint16_t batteryVoltage) {
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
    lcdSetCursor(originalRow, originalCol);
}

void batteryIndicatorEnable() {
    enabled = 1;
}

void batteryIndicatorDisable() {
    enabled = 0;
}
