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

typedef enum {
    ColorIndex_Red = 0,
    ColorIndex_Blue = 1,
    ColorIndex_Clear = 2,
    ColorIndex_Green = 3,
    ColorIndex_Invalid = 4,
} ColorIndex;

typedef struct {
    GPIO_TypeDef *s0Port;
    GPIO_TypeDef *s1Port;
    GPIO_TypeDef *s2Port;
    GPIO_TypeDef *s3Port;
    uint16_t s0Pin;
    uint16_t s1Pin;
    uint16_t s2Pin;
    uint16_t s3Pin;
    volatile uint16_t measuredVal[4];
    ColorIndex currentCaptureIndex;
    uint16_t numOfAverages;
    uint16_t itCntr;
    uint16_t prevCaptureVal;
    uint32_t captureSum;
} ColorSensor;

typedef struct {
    uint16_t h; // range: 0-360
    uint8_t s;  // range: 0-100
    uint8_t v;  // range: 0-100
} ColorHsv;

void colorSensorInit(ColorSensor* cs,
                     GPIO_TypeDef *csS0Port, uint16_t csS0Pin, GPIO_TypeDef *csS1Port, uint16_t csS1Pin,
                     GPIO_TypeDef *csS2Port, uint16_t csS2Pin, GPIO_TypeDef *csS3Port, uint16_t csS3Pin,
                     uint16_t inputFiltering);
void colorSensorCaptureHandler(ColorSensor* cs, uint16_t captureVal);
void colorSensorGetPeriods(ColorSensor* cs, uint16_t* capRed, uint16_t* capGreen, uint16_t* capBlue, uint16_t* capClear);
void colorSensorGetRgb(ColorSensor* cs, uint8_t* r, uint8_t* g, uint8_t* b);
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, uint16_t* hue, uint8_t* sat, uint8_t* val);
void colorSensorGetHsv(ColorSensor* cs, ColorHsv* color);

#endif /* COLOR_SENSOR_COLOR_SENSOR_H_ */
