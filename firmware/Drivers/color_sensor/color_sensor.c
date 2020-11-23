/*
 * color_sensor.c
 *
 *  Created on: Oct 21, 2020
 *      Author: ksstms
 *
 * Driver for the TCS3200 color sensor
 */
#include "color_sensor.h"

/**
 * Set the color filter of the sensor
 * @param colorIndex
 */
void setColorFilter(volatile ColorSensor* cs, ColorIndex colorIndex) {
    HAL_GPIO_WritePin(cs->s2Port, cs->s2Pin, colorIndex & (1 << 1));
    HAL_GPIO_WritePin(cs->s3Port, cs->s3Pin, colorIndex & (1 << 0));
}

/**
 * Call this from the sensor output's input capture interrupt handler
 * @param captureVal timer's input capture value
 */
void colorSensorCaptureHandler(volatile ColorSensor* cs, uint16_t captureVal) {
    uint16_t elapsed = captureVal - cs->prevCaptureVal;

    cs->prevCaptureVal = captureVal;
    cs->captureSum += elapsed;
    cs->itCntr++;

    if (cs->itCntr == cs->numOfAverages) {
        cs->itCntr = 0;

        cs->measuredVal[cs->currentCaptureIndex] = cs->captureSum / 16;
        cs->captureSum = 0;

        cs->currentCaptureIndex++;
        if (cs->currentCaptureIndex == ColorIndex_Invalid) {
            cs->currentCaptureIndex = ColorIndex_Red;
        }

        setColorFilter(cs, cs->currentCaptureIndex);
    }

}

/**
 * Initialize the color sensor
 * @param csSxPort GPIO port of Sx setting pin
 * @param csSxPin pin number of Sx setting pin
 * @param inputFiltering number of capture values to average for each channel
 */
void colorSensorInit(volatile ColorSensor* cs,
                     GPIO_TypeDef *csS0Port, uint16_t csS0Pin, GPIO_TypeDef *csS1Port, uint16_t csS1Pin,
                     GPIO_TypeDef *csS2Port, uint16_t csS2Pin, GPIO_TypeDef *csS3Port, uint16_t csS3Pin,
                     uint16_t inputFiltering) {
    cs->s0Port = csS0Port;
    cs->s1Port = csS1Port;
    cs->s2Port = csS2Port;
    cs->s3Port = csS3Port;
    cs->s0Pin = csS0Pin;
    cs->s1Pin = csS1Pin;
    cs->s2Pin = csS2Pin;
    cs->s3Pin = csS3Pin;

    cs->numOfAverages = inputFiltering;

    // Disable frequency scaling
    HAL_GPIO_WritePin(cs->s0Port, cs->s0Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs->s1Port, cs->s1Pin, GPIO_PIN_SET);

    // Set color filter
    setColorFilter(cs, ColorIndex_Red);
    cs->currentCaptureIndex = ColorIndex_Red;
}

/**
 * Read the raw capture period length of each channel
 * @param capRed
 * @param capGreen
 * @param capBlue
 * @param capClear
 */
void colorSensorGetPeriods(volatile ColorSensor* cs, uint16_t* capRed, uint16_t* capGreen,
                           uint16_t* capBlue, uint16_t* capClear) {
    *capRed   = cs->measuredVal[ColorIndex_Red];
    *capGreen = cs->measuredVal[ColorIndex_Green];
    *capBlue  = cs->measuredVal[ColorIndex_Blue];
    *capClear = cs->measuredVal[ColorIndex_Clear];
}
