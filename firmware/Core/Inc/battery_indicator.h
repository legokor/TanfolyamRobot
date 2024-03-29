/*
 * battery_indicator.h
 *
 *  Created on: Nov 15, 2020
 *      Author: ksstms
 */

#ifndef INC_BATTERY_INDICATOR_H_
#define INC_BATTERY_INDICATOR_H_

#include <stdint.h>

void batteryIndicatorInit();
void batteryIndicatorDisplay(uint8_t row, uint8_t col, uint16_t batteryVoltage);
void batteryIndicatorEnable();
void batteryIndicatorDisable();

#endif /* INC_BATTERY_INDICATOR_H_ */
