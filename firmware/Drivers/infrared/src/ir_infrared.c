/*
 * ir_infrared.c
 *
 *  Created on: Apr 15, 2024
 *      Author: serbanandi
 *
 * Driver for GY-53 Infrared ranging module.
 *
 * A form of output data for the module is the PWM output, which has a square wave period of 20 Hz.
 * The high level corresponds to the measured distance
 * Formula: Distance (mm) = High time (ms) * 100 = High time (us)/10
 * For example, if the measured high time is 10000us, then Distance=10000/10=1000mm
 *
 * The echo signal measurement is based on the timer input capture function. STM32 timer channels don't
 * support capturing on both rising and falling edges of an input, so two timer channels must be used:
 * one in direct mode and one in indirect mode. In indirect mode the channel's edge detector is routed
 * to the neighboring channel's input, so only one external connection is necessary.
 */
#include "ir_interface.h"

// See specification above
#define IR_US_TO_MM_DIV 	10

/**
 * Initialize the InfraRed struct
 * @param ir empty InfraRed struct
 * @param captureTimer timer used for input capture of the sensor's echo signal
 * @param captureTimerFrequencyHz frequency of the captureTimer
 */
void ir_init(ir_InfraRed* ir, TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequencyHz) {
    ir->captureTimer = captureTimer;

    // Calculate the time period of the capture timer
    uint16_t frequencyKHz = captureTimerFrequencyHz / 1000;
    uint16_t periodNs = 1000*1000 / frequencyKHz;
    ir->captureTimerPeriodNs = periodNs;

    ir->pwmIsHigh = 0;
    ir->lastDistance = 0;
}



/**
 * Call this from a timer's capture interrupt on rising edges of the echo signal
 * @param ir
 * @param captureVal the captured value from the timer channel
 */
void ir_handlerRisingCapture(ir_InfraRed* ir, uint16_t captureVal) {
    if (!ir->pwmIsHigh) {
        ir->captureStart = captureVal;
        ir->pwmIsHigh = 1;
    }
}





/**
 * Call this from a timer's capture interrupt on falling edges of the echo signal
 * @param ir
 * @param captureVal the captured value from the timer channel
 */
void ir_handlerFallingCapture(ir_InfraRed* ir, uint16_t captureVal) {
    if (ir->pwmIsHigh) {
        uint16_t captureStop = captureVal;
        ir->pwmIsHigh = 0;

        uint16_t pwmWidthTicks = captureStop - ir->captureStart;
		uint32_t pwmWidthUs = (pwmWidthTicks * ir->captureTimerPeriodNs) / 1000;
		ir->lastDistance = pwmWidthUs / IR_US_TO_MM_DIV;
    }
}


/**
 * Get the last measured distance in mm
 * @param ir
 * @return  -1 if the measurement hasn't finished yet
 *         >=0 if the measurement is done
 */
uint16_t ir_getDistance(ir_InfraRed* ir) {
    return ir->lastDistance;
}



