/*
 * color_sensor.h
 *
 *  Created on: Oct 21, 2020
 *      Author: ksstms
 */

#ifndef COLOR_SENSOR_COLOR_SENSOR_H_
#define COLOR_SENSOR_COLOR_SENSOR_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

void colorSensorInit(GPIO_TypeDef *csS0Port, uint16_t csS0Pin, GPIO_TypeDef *csS1Port, uint16_t csS1Pin,
                     GPIO_TypeDef *csS2Port, uint16_t csS2Pin, GPIO_TypeDef *csS3Port, uint16_t csS3Pin,
                     uint16_t inputFiltering);
void colorSensorCaptureHandler(uint16_t captureVal);
void colorSensorGetPeriods(uint16_t* capRed, uint16_t* capGreen, uint16_t* capBlue, uint16_t* capClear);

#endif /* COLOR_SENSOR_COLOR_SENSOR_H_ */
