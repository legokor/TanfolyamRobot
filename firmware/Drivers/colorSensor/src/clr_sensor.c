/*
 * clr_sensor.c
 *
 *  Created on: Oct 21, 2020
 *      Author: ksstms
 *
 * Driver for the TCS3200 color sensor
 */
#include "clr_interface.h"
#include "clr_calibration.h"

/**
 * Set the color filter of the sensor
 * @param colorIndex
 */
static void clr_setColorFilter(clr_ColorSensor* cs, clr_ColorIndex colorIndex) {
    HAL_GPIO_WritePin(cs->s2Port, cs->s2Pin, colorIndex & (1 << 1));
    HAL_GPIO_WritePin(cs->s3Port, cs->s3Pin, colorIndex & (1 << 0));
}

/**
 * Call this from the sensor output's input capture interrupt handler
 * @param captureVal timer's input capture value
 */
void clr_captureHandler(clr_ColorSensor* cs, uint16_t captureVal) {
    uint16_t elapsed = captureVal - cs->prevCaptureVal;

    cs->prevCaptureVal = captureVal;
    cs->captureSum += elapsed;
    cs->itCntr++;

    if (cs->itCntr == cs->numOfAverages) {
        cs->itCntr = 0;

        cs->measuredVal[cs->currentCaptureIndex] = cs->captureSum / cs->numOfAverages;
        cs->captureSum = 0;

        cs->currentCaptureIndex++;
        if (cs->currentCaptureIndex == ColorIndex_Invalid) {
            cs->currentCaptureIndex = ColorIndex_Red;
        }

        clr_setColorFilter(cs, cs->currentCaptureIndex);
    }

}

/**
 * Initialize the color sensor
 * @param csSxPort GPIO port of Sx setting pin
 * @param csSxPin pin number of Sx setting pin
 * @param inputFiltering number of capture values to average for each channel
 */
void clr_init(clr_ColorSensor* cs,
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
    clr_setColorFilter(cs, ColorIndex_Red);
    cs->currentCaptureIndex = ColorIndex_Red;
}

/**
 * Read the raw capture period length of each channel
 * @param capRed
 * @param capGreen
 * @param capBlue
 * @param capClear
 */
void clr_getPeriods(clr_ColorSensor* cs, uint16_t* capRed, uint16_t* capGreen,
                           uint16_t* capBlue, uint16_t* capClear) {
    *capRed   = cs->measuredVal[ColorIndex_Red];
    *capGreen = cs->measuredVal[ColorIndex_Green];
    *capBlue  = cs->measuredVal[ColorIndex_Blue];
    *capClear = cs->measuredVal[ColorIndex_Clear];
}

/**
 * Read calibrated RGB values
 * @param r
 * @param g
 * @param b
 */
void clr_getRgb(clr_ColorSensor* cs, uint8_t* r, uint8_t* g, uint8_t* b) {
    uint16_t pR, pG, pB, pC;
    clr_getPeriods(cs, &pR, &pG, &pB, &pC);

    // Set to max value if we divide by 0
    uint16_t sR = 65535;
    uint16_t sG = 65535;
    uint16_t sB = 65535;

    if (pR != 0) {
        sR = SCALE / pR;
    }
    if (pG != 0) {
        sG = SCALE / pG;
    }
    if (pB != 0) {
        sB = SCALE / pB;
    }

    *r = (sR - R_MIN) * 255 / (R_MAX - R_MIN);
    *g = (sG - G_MIN) * 255 / (G_MAX - G_MIN);
    *b = (sB - B_MIN) * 255 / (B_MAX - B_MIN);
}

/**
 * Convert an RGB color value to HSV color model
 * @param r
 * @param g
 * @param b
 * @param hue
 * @param sat
 * @param val
 */
void clr_rgb2hsv(uint8_t r, uint8_t g, uint8_t b, uint16_t* hue, uint8_t* sat, uint8_t* val){
    uint8_t max, min;
    float hue_f;

    min = (r < g) ? r : g;
    max = (r > g) ? r : g;
    min = (min < b) ? min : b;
    max = (max > b) ? max : b;

    *val = (uint16_t)max * 100 / 255;

    uint8_t delta = max - min;

    if (delta == 0) {
        *sat = 0;
        *hue = 0;
    } else {
        *sat = 100 * delta / max;

        if (max == r) {
            hue_f = 60 * ( ( ((float)g - (float)b) / delta )    );    // TODO: optimize
        } else if (max == g) {
            hue_f = 60 * ( ( ((float)b - (float)r) / delta ) + 2);    // TODO: optimize
        } else {
            hue_f = 60 * ( ( ((float)r - (float)g) / delta ) + 4);    // TODO: optimize
        }
        if (hue_f < 0) {
                hue_f += 360;
		}
        *hue = (uint16_t)hue_f % 360;
    }

    
    // TODO: is this necessary? Isn't the output of % always +?

}

/**
 * Read calibrated color in HSV
 * @param color
 */
void clr_getHsv(clr_ColorSensor* cs, clr_ColorHsv* color) {
    uint8_t r, g, b;
    clr_getRgb(cs, &r, &g, &b);
    clr_rgb2hsv(r, g, b, &(color->h), &(color->s), &(color->v));
}
