/*
 * us_ultrasonic.c
 *
 *  Created on: Nov 3, 2020
 *      Author: ksstms, dkiovics
 *
 * Driver for HC-SR04, HY-SRF05, and similar ultrasonic ranging modules.
 *
 * The measurement is started by a pulse on the trigger pin. The module outputs
 * an echo pulse with the length of the roundtrip time of the ultrasonic pulses.
 *
 * The echo signal measurement is based on the timer input capture function. STM32 timer channels don't
 * support capturing on both rising and falling edges of an input, so two timer channels must be used:
 * one in direct mode and one in indirect mode. In indirect mode the channel's edge detector is routed
 * to the neighboring channel's input, so only one external connection is necessary.
 */
#include "us_interface.h"

#define TIMEOUT 60000

// Approximate width of the trigger impulse. This has to be >= 10us
#define US_TRIGGER_WIDTH	15

// Speed of sound in air is about 340 m/s, so the width of the echo signal is 29 us/cm. The distance is
// measure in both directions, so we need to double that.
#define US_CM_DIVIDER 		58

// How often the distance is measured in the background (1 overflow is roughly 33ms)
#define TIM_OVERFLOW_CNT 	3

/**
 * Initialize the UltraSonic struct
 * @param us empty UltraSonic struct
 * @param triggerPort GPIO port of the sensor's trigger signal
 * @param triggerPin GPIO pin of the sensor's trigger signal
 * @param captureTimer timer used for input capture of the sensor's echo signal
 * @param captureTimerFrequencyHz frequency of the captureTimer
 * @param delayTimer timer used for delays
 * @param delayTimerFrequencyHz frequency of the delayTimer
 */
void us_init(us_UltraSonic* us, GPIO_TypeDef* triggerPort, uint16_t triggerPin,
            TIM_HandleTypeDef* captureTimer, uint32_t captureTimerFrequencyHz,
            TIM_HandleTypeDef* delayTimer, uint32_t delayTimerFrequencyHz) {
    us->triggerPort = triggerPort;
    us->triggerPin = triggerPin;

    us->captureTimer = captureTimer;
    us->delayTimer = delayTimer;

    // Calculate the number of timer pulses for the required trigger pulse width
    uint16_t frequencyKHz = delayTimerFrequencyHz / 1000;
    uint16_t periodNs = 1000*1000 / frequencyKHz;
    us->delayTimerPeriodNs = periodNs;

    // Calculate the time period of the capture timer
    frequencyKHz = captureTimerFrequencyHz / 1000;
    periodNs = 1000*1000 / frequencyKHz;
    us->captureTimerPeriodNs = periodNs;

    us->measurementValid = 0;
    us->echoIsHigh = 0;
    us->pulseActive = 0;
    us->lastDistance = 0;
    us->timerCounter = 0;
    HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_RESET);
}



/**
 * Call this from a timer's capture interrupt on rising edges of the echo signal
 * @param us
 * @param captureVal the captured value from the timer channel
 */
void us_handlerRisingCapture(us_UltraSonic* us, uint16_t captureVal) {
    if (us->measurementValid && !us->echoIsHigh) {
        us->captureStart = captureVal;
        us->echoIsHigh = 1;
    }
}





/**
 * Call this from a timer's capture interrupt on falling edges of the echo signal
 * @param us
 * @param captureVal the captured value from the timer channel
 */
void us_handlerFallingCapture(us_UltraSonic* us, uint16_t captureVal) {
    if (us->measurementValid && us->echoIsHigh) {
        uint16_t captureStop = captureVal;
        us->echoIsHigh = 0;
        us->measurementValid = 0;

        uint16_t echoWidthTicks = captureStop - us->captureStart;
		uint32_t echoWidthUs = (echoWidthTicks * us->captureTimerPeriodNs) / 1000;
		us->lastDistance = echoWidthUs / US_CM_DIVIDER;
    }
}

/**
 * Start an async measurement pulse
 * @param us
 */
void us_startMeasurementPulseAsync(us_UltraSonic* us) {
	us->timerCounter++;
	if(us->timerCounter==TIM_OVERFLOW_CNT){
		us->timerCounter = 0;

		uint16_t triggerDelay = (US_TRIGGER_WIDTH * 1000) / us->delayTimerPeriodNs;
		us->delayTimer->Instance->CCR4 = us->delayTimer->Instance->CNT + triggerDelay;

		us->measurementValid = 0;
		us->pulseActive = 1;
		HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_SET);
	}
}

/**
 * End an async measurement pulse
 * @param us
 */
void usEndMeasurementPulseAsync(us_UltraSonic* us) {
	HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_RESET);

	us->echoIsHigh = 0;
	us->measurementValid = 1;
	us->pulseActive = 0;

	us->delayTimer->Instance->CCR4 = us->captureTimer->Instance->CNT + TIMEOUT;
}

/**
 * Handle compare event (end pulse signal or timeout)
 * @param us
 */
void us_handleCompareAsync(us_UltraSonic* us){
	if(us->pulseActive){
		usEndMeasurementPulseAsync(us);
	}else{
		us->measurementValid = 0;
	}
}

/**
 * Get the last measured distance in cm
 * @param us
 * @return  -1 if the measurement hasn't finished yet
 *         >=0 if the measurement is done
 */
uint16_t us_getDistance(us_UltraSonic* us) {
    return us->lastDistance;
}



