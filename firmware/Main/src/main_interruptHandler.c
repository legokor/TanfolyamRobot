/*
 * main_interruptHandler.c
 *
 *  Created on: Jan 28, 2025
 *      Author: dkiovics
 */

#include "main.h"
#include "main_interface.h"
#include "main_config.h"
#include "tel_interface.h"
#include "lcd_interface.h"

static volatile uint16_t batteryVoltage = 0;
static volatile char espState = 0;
static volatile uint8_t batteryAdcBusy = 0;
extern volatile uint8_t main_usDelayOverflow;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!main_robotInstance.initCplt)
    	return;

    if (htim == LCD_IMU_ENCODER_TIMER)
    {
    	enc_timerOverflowHandler(&main_robotInstance.rightEncoder);
    	enc_timerOverflowHandler(&main_robotInstance.leftEncoder);

    	lcd_handler();

    	static int imuCnt = 0;
    	imuCnt++;
		if (imuCnt == 78)	 			//Every <5ms (>200Hz)
		{
			imuCnt = 0;
			mpu_timPeriodEllapsedCallback(&main_robotInstance.imu);
		}

		static int orCnt = 0;
		orCnt++;
		if (orCnt == 130)				//Every 8.125ms
		{
			orCnt = 0;
			mpu_updateOrientation(&main_robotInstance.imu, &main_robotInstance.orientation, 0.008125f);
		}
    }

	if (htim == MOTOR_CONTROL_TIMER)
    {
    	static int scCnt = 0;
    	scCnt++;
        if (scCnt == MOTOR_CONTROL_PRESCALE)
        {
        	scCnt = 0;
			drv_speedControlHandler(&main_robotInstance.rightSpeedCtrl);
			drv_speedControlHandler(&main_robotInstance.leftSpeedCtrl);
        }
    }

    if (htim == SERVO_DELAYUS_TIMER)
    {
    	main_usDelayOverflow = 1;
    }

    if (htim == VBAT_ADC_TIMER)
    {
        static int adcCnt = 0;
        adcCnt++;
        if (adcCnt == BATTERY_INDICATOR_PERIOD)
        {
        	adcCnt = 0;
            lcd_displayStatus(BATTERY_INDICATOR_ROW, BATTERY_INDICATOR_COL, batteryVoltage, espState);
        }

        if (!batteryAdcBusy)
        {
            batteryAdcBusy = 1;
            HAL_ADC_Start_IT(VBAT_ADC);
        }
    }

	if (htim == RANGE_COLOR_ESP_TIMER)
	{
#if US_SENSOR
		us_startMeasurementPulseAsync(&main_robotInstance.us);
#endif
    	static int telCnt = 0;
		telCnt++;
    	if (telCnt == 10)
    	{
    		telCnt = 0;
    		tel_sendDataToEsp();
    	}
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(!main_robotInstance.initCplt)
	    return;

    if (hadc == VBAT_ADC)
    {
        uint16_t adcVal = HAL_ADC_GetValue(VBAT_ADC);
        batteryVoltage =  adcVal * ADC_TO_VBAT_MULTIPLIER + ADC_TO_VBAT_OFFSET;
        batteryAdcBusy = 0;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(!main_robotInstance.initCplt)
	    return;

	if(htim == RANGE_COLOR_ESP_TIMER)
	{
        switch (htim->Channel)
        {
#if US_SENSOR
            case US_RISING_ACTIVE_CHANNEL:
                us_handlerRisingCapture(&main_robotInstance.us, HAL_TIM_ReadCapturedValue(htim, US_RISING_CHANNEL));
                break;
            case US_FALLING_ACTIVE_CHANNEL:
                us_handlerFallingCapture(&main_robotInstance.us, HAL_TIM_ReadCapturedValue(htim, US_FALLING_CHANNEL));
                break;
#elif IR_SENSOR
            case IR_RISING_ACTIVE_CHANNEL:
                ir_handlerRisingCapture(&main_robotInstance.ir, HAL_TIM_ReadCapturedValue(htim, IR_RISING_CHANNEL));
                break;
            case IR_FALLING_ACTIVE_CHANNEL:
                ir_handlerFallingCapture(&main_robotInstance.ir, HAL_TIM_ReadCapturedValue(htim, IR_FALLING_CHANNEL));
                break;
#endif
            case COLOR_ACTIVE_CHANNEL:
                clr_captureHandler(&main_robotInstance.colorSensor, HAL_TIM_ReadCapturedValue(htim, COLOR_CHANNEL));
                break;
            default:
            	break;
        }
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(!main_robotInstance.initCplt)
	    return;

#if US_SENSOR
    if (htim == RANGE_COLOR_ESP_TIMER && htim->Channel == US_ASYNC_ACTIVE_CHANNEL)
    {
    	us_handleCompareAsync(&main_robotInstance.us);
    }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	txt_handleReceiveCplt(&main_robotInstance.usbUart, USB_UART, main_robotInstance.initCplt);
	char c = txt_handleReceiveCplt(&main_robotInstance.espUart, ESP_UART, main_robotInstance.initCplt && main_robotInstance.espReady);
	if(c)
		espState = c;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!main_robotInstance.initCplt)
	    return;

	if (huart == USB_UART)
	{
		txt_handleTransmitCplt(&main_robotInstance.usbUart, huart);
	}
	if (huart == ESP_UART)
	{
		txt_handleTransmitCplt(&main_robotInstance.espUart, huart);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(!main_robotInstance.initCplt)
		return;

	if (hi2c == IMU_I2C)
	{
		mpu_i2cReceiveCpltCallback(&main_robotInstance.imu);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(!main_robotInstance.initCplt)
	    return;

    switch (GPIO_Pin)
    {
        case ENC1_A_Pin:
        	enc_handlerA(&main_robotInstance.rightEncoder);
        	break;
        case ENC2_A_Pin:
        	enc_handlerA(&main_robotInstance.leftEncoder);
        	break;
    }
}

