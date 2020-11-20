/*
 * ultrasonic.c
 *
 *  Created on: Nov 3, 2020
 *      Author: ksstms
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
#include "ultrasonic.h"

// Approximate width of the trigger impulse. This has to be >=10 us
const uint8_t triggerWidthUs = 15;

// Speed of sound in air is about 340 m/s, so the width of the echo signal is 29 us/cm. The distance is
// measure in both directions, so we need to double that.
const uint8_t usToCmDivider = 58;

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
void usInit(volatile UltraSonic* us, GPIO_TypeDef* triggerPort, uint16_t triggerPin,
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

    us->busy = 0;
    HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_RESET);
}

/**
 * Call this from a timer's capture interrupt on rising edges of the echo signal
 * @param us
 * @param captureVal the captured value from the timer channel
 */
void usHandlerRisingCapture(volatile UltraSonic* us, uint16_t captureVal) {
    if (us->busy) {
        us->captureStart = captureVal;
    }
}

/**
 * Call this from a timer's capture interrupt on falling edges of the echo signal
 * @param us
 * @param captureVal the captured value from the timer channel
 */
void usHandlerFallingCapture(volatile UltraSonic* us, uint16_t captureVal) {
    if (us->busy) {
        us->captureStop = captureVal;
        us->busy = 0;
    }
}

/**
 * Send trigger signal to the sensor
 * @note this function blocks for triggerWidthUs
 * @param us
 */
void usStartMeasurement(volatile UltraSonic* us) {
    if (us->busy) {
        return;
    }

    us->busy = 1;

    // Calculate trigger pulse width in timer periods
    uint16_t triggerDelay = (triggerWidthUs * 1000) / us->delayTimerPeriodNs;

    // Start trigger pulse
    HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_SET);

    // Delay
    uint16_t startTick = us->delayTimer->Instance->CNT;
    uint16_t delta=0;

    while (delta <= triggerDelay) {
        delta = us->delayTimer->Instance->CNT - startTick;
    }

    // Stop trigger pulse
    HAL_GPIO_WritePin(us->triggerPort, us->triggerPin, GPIO_PIN_RESET);
}

/**
 * Get the last measured distance in cm
 * @param us
 * @return  -1 if the measurement hasn't finished yet
 *         >=0 if the measurement is done
 */
int16_t usGetDistance(volatile UltraSonic* us) {
    if (us->busy) {
        return -1;
    }

    uint16_t echoWidthTicks = us->captureStop - us->captureStart;
    uint32_t echoWidthUs = (echoWidthTicks * us->captureTimerPeriodNs) / 1000;

    return echoWidthUs / usToCmDivider;
}

/**
 * Start a measurement and wait until it completes
 * @param us
 * @return distance in cm
 */
uint16_t usMeasureBlocking(volatile UltraSonic* us) {
    usStartMeasurement(us);

    // Wait for completion
    // TODO: add timeout
    int16_t distance;
    do {
        distance = usGetDistance(us);
    } while (distance < 0);

    return (uint16_t)distance;
}

