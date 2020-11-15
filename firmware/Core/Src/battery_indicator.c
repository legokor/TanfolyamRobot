/*
 * battery_indicator.c
 *
 *  Created on: Nov 15, 2020
 *      Author: ksstms
 */
#include "battery_indicator.h"
#include "lcd.h"

#define BATTERY_CELL_COUNT 2

/**
 * Custom characters for HD44780-based LCD
 */
uint8_t batteryChar[][8] = {
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
uint16_t voltageThreshold[] = { BATTERY_CELL_COUNT * 3450,  // 5%
                                BATTERY_CELL_COUNT * 3680,  // 10%
                                BATTERY_CELL_COUNT * 3770,  // 30%
                                BATTERY_CELL_COUNT * 3820,  // 50%
                                BATTERY_CELL_COUNT * 3920,  // 70%
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

    lcdPutc(row, col, c);
}
