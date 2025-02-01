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
 * WARNING: the delay time might vary +-US_DELAY_TIMER_PERIOD on very rare occasions
 */
void main_delayUs(uint32_t us)
{
	main_usDelayOverflow = 0;
	uint32_t now = SERVO_DELAYUS_TIMER->Instance->CNT;
	uint32_t end = now + us;

	if (us >= US_DELAY_TIMER_PERIOD)
	{
		uint32_t overflowCnt = end / US_DELAY_TIMER_PERIOD;
		uint32_t timerTicks = end % US_DELAY_TIMER_PERIOD;

		while(overflowCnt)
		{
			if (main_usDelayOverflow)
			{
				main_usDelayOverflow = 0;
				overflowCnt--;
			}
		}
		while (!main_usDelayOverflow && SERVO_DELAYUS_TIMER->Instance->CNT < timerTicks) { }
	}
	else
	{
		if (end >= US_DELAY_TIMER_PERIOD)
		{
			end -= US_DELAY_TIMER_PERIOD;
			while (!main_usDelayOverflow) { }
			main_usDelayOverflow = 0;
		}
		while (!main_usDelayOverflow && SERVO_DELAYUS_TIMER->Instance->CNT < end) { }
	}
}
