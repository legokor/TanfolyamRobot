/*
 * color_sensor.c
 *
 *  Created on: Oct 21, 2020
 *      Author: ksstms
 *
 * Driver for the TCS3200 color sensor
 */
#include "color_sensor.h"

typedef enum {
    ColorIndex_Red = 0,
    ColorIndex_Blue = 1,
    ColorIndex_Clear = 2,
    ColorIndex_Green = 3,
    ColorIndex_Invalid = 4,
} ColorIndex;

/*
 * Color sensor pin locations
 */
GPIO_TypeDef *s0Port;
GPIO_TypeDef *s1Port;
GPIO_TypeDef *s2Port;
GPIO_TypeDef *s3Port;
uint16_t s0Pin;
uint16_t s1Pin;
uint16_t s2Pin;
uint16_t s3Pin;

/*
 * Capture values for each channel
 */
volatile uint16_t measuredVal[4] = {0, 0, 0, 0};
volatile ColorIndex currentCaptureIndex;
uint16_t numOfAverages;

/**
 * Set the color filter of the sensor
 * @param colorIndex
 */
void setColorFilter(ColorIndex colorIndex) {
    HAL_GPIO_WritePin(s2Port, s2Pin, colorIndex & (1 << 1));
    HAL_GPIO_WritePin(s3Port, s3Pin, colorIndex & (1 << 0));
}

/**
 * Call this from the sensor output's input capture interrupt handler
 * @param captureVal timer's input capture value
 */
void colorSensorCaptureHandler(uint16_t captureVal) {
    static uint16_t prevCaptureVal;
    static uint32_t itCntr = 0;
    static uint32_t captureSum = 0;

    uint16_t elapsed = captureVal - prevCaptureVal;

    prevCaptureVal = captureVal;

    captureSum += elapsed;
    itCntr++;

    if (itCntr == numOfAverages) {
        itCntr = 0;

        measuredVal[currentCaptureIndex] = captureSum / 16;
        captureSum = 0;

        currentCaptureIndex++;
        if (currentCaptureIndex == ColorIndex_Invalid) {
          currentCaptureIndex = ColorIndex_Red;
        }

        setColorFilter(currentCaptureIndex);
    }

}

/**
 * Initialize the color sensor
 * @param csSxPort GPIO port of Sx setting pin
 * @param csSxPin pin number of Sx setting pin
 * @param inputFiltering number of capture value to average for each channel
 */
void colorSensorInit(GPIO_TypeDef *csS0Port, uint16_t csS0Pin, GPIO_TypeDef *csS1Port, uint16_t csS1Pin,
                     GPIO_TypeDef *csS2Port, uint16_t csS2Pin, GPIO_TypeDef *csS3Port, uint16_t csS3Pin,
                     uin16_t inputFiltering) {
    s0Port = csS0Port;
    s1Port = csS1Port;
    s2Port = csS2Port;
    s3Port = csS3Port;
    s0Pin = csS0Pin;
    s1Pin = csS1Pin;
    s2Pin = csS2Pin;
    s3Pin = csS3Pin;

    numOfAverages = inputFiltering;

    /*
     * Disable frequency scaling
     */
    HAL_GPIO_WritePin(s0Port, s0Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(s1Port, s1Pin, GPIO_PIN_SET);

    /*
     * Set color filter
     */
    setColorFilter(ColorIndex_Red);
    currentCaptureIndex = ColorIndex_Red;
}

/**
 * Read the raw capture period length of each channel
 * @param capRed
 * @param capGreen
 * @param capBlue
 * @param capClear
 */
void colorSensorGetPeriods(uint16_t* capRed, uint16_t* capGreen, uint16_t* capBlue, uint16_t* capClear) {
    *capRed   = measuredVal[ColorIndex_Red];
    *capGreen = measuredVal[ColorIndex_Green];
    *capBlue  = measuredVal[ColorIndex_Blue];
    *capClear = measuredVal[ColorIndex_Clear];
}
