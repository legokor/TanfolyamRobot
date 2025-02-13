/*
 * main_usDelay.c
 *
 *  Created on: Feb 1, 2025
 *      Author: dkiovics
 */

#include "main_config.h"
#include "stm32f1xx_hal.h"

volatile uint8_t main_usDelayOverflow = 0;


/**
 * This function blocks for a certain number of microseconds
 * WARNING: the delay time might can't be greater than US_DELAY_TIMER_PERIOD - 2000
 */
void main_delayUs(uint32_t us)
{
	if (us > US_DELAY_TIMER_PERIOD - 2000)
		us = US_DELAY_TIMER_PERIOD - 2000;
	uint32_t now = SERVO_DELAYUS_TIMER->Instance->CNT;
	while (1)
	{
		int32_t diff = (int32_t)SERVO_DELAYUS_TIMER->Instance->CNT - now;
		if(diff < 0)
			diff += US_DELAY_TIMER_PERIOD;
		if(diff > us)
			return;
	}
}
