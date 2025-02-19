/*
 * main_config.h
 *
 *  Created on: Sep 9, 2024
 *      Author: dkiovics
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/**************** Timer stuff ****************/
#define LCD_IMU_ENCODER_TIMER       (&htim1)
#define MOTOR12_PWM1_TIMER         	(&htim1)
#define MOTOR_CONTROL_TIMER       	(&htim1)
#define SERVO_DELAYUS_TIMER			(&htim2)
#define MOTOR12_PWM2_TIMER         	(&htim3)
#define VBAT_ADC_TIMER           	(&htim4)
#define RANGE_COLOR_ESP_TIMER		(&htim4)

#define US_DELAY_TIMER_PERIOD		20000

#define MOTOR_CONTROL_PRESCALE    	8

#define MOTOR1_PWM1_TIMER_CHANNEL 	TIM_CHANNEL_1
#define MOTOR1_PWM1_TIMER_PERIOD  	4500
#define MOTOR1_PWM1_OUTPUT_TYPE   	PwmOutput_N
#define MOTOR1_PWM2_TIMER_CHANNEL 	TIM_CHANNEL_3
#define MOTOR1_PWM2_TIMER_PERIOD  	4500
#define MOTOR1_PWM2_OUTPUT_TYPE   	PwmOutput_P
#define MOTOR1_REVERSED           	1

#define MOTOR2_PWM1_TIMER_CHANNEL 	TIM_CHANNEL_2
#define MOTOR2_PWM1_TIMER_PERIOD  	4500
#define MOTOR2_PWM1_OUTPUT_TYPE   	PwmOutput_N
#define MOTOR2_PWM2_TIMER_CHANNEL 	TIM_CHANNEL_4
#define MOTOR2_PWM2_TIMER_PERIOD  	4500
#define MOTOR2_PWM2_OUTPUT_TYPE   	PwmOutput_P
#define MOTOR2_REVERSED           	0

#define MOTOR1_ENCODER_RESOLUTION 	EncoderResolution_1
#define MOTOR2_ENCODER_RESOLUTION 	EncoderResolution_1
#define MOTOR_ENCODER_MAX_SPEED_CPS 3000
#define MOTOR_ENCODER_TIMER_FREQ	72000000

#define SERVO_CHANNEL        		TIM_CHANNEL_3
#define SERVO_ACTIVE_CHANNEL 		HAL_TIM_ACTIVE_CHANNEL_3
#define SERVO_PWM_INVERTED         	1
#define SERVO_PWM_PERIOD       		20000
#define SERVO_START_POS         	680
#define SERVO_END_POS           	2440
#define SERVO_INIT_POS 				0

#if US_SENSOR
	#define US_TIMER_FREQUENCY_HZ      	(2*1000*1000)
	#define US_RISING_CHANNEL          	TIM_CHANNEL_1
	#define US_RISING_ACTIVE_CHANNEL   	HAL_TIM_ACTIVE_CHANNEL_1
	#define US_FALLING_CHANNEL         	TIM_CHANNEL_2
	#define US_FALLING_ACTIVE_CHANNEL  	HAL_TIM_ACTIVE_CHANNEL_2
	#define US_ASYNC_ACTIVE_CHANNEL    	HAL_TIM_ACTIVE_CHANNEL_4
	#define US_ASYNC_CHANNEL    	   	TIM_CHANNEL_4
#elif IR_SENSOR
	#define IR_TIMER_FREQUENCY_HZ      (2*1000*1000)
	#define IR_RISING_CHANNEL          TIM_CHANNEL_1
	#define IR_RISING_ACTIVE_CHANNEL   HAL_TIM_ACTIVE_CHANNEL_1
	#define IR_FALLING_CHANNEL         TIM_CHANNEL_2
	#define IR_FALLING_ACTIVE_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_2
#else
	#error "No ranging module defined as active"
#endif

#define COLOR_CHANNEL              	TIM_CHANNEL_3
#define COLOR_ACTIVE_CHANNEL       	HAL_TIM_ACTIVE_CHANNEL_3


/**************** STM32 social skills ****************/
#define USB_UART 					(&huart1)
#define USB_UART_IR 				USART1_IRQn
#define USB_UART_DMA_IR 			DMA1_Channel4_IRQn

#define ESP_UART 					(&huart3)
#define ESP_UART_IR 				USART3_IRQn
#define ESP_UART_DMA_IR 			DMA1_Channel2_IRQn

#define IMU_I2C 					(&hi2c2)
#define IMU_I2C_DMA_IR 				DMA1_Channel5_IRQn
#define IMU_I2C_IT_IR 				I2C2_EV_IRQn
#define IMU_GYRO_OFFSET_EN 			1


/**************** Miscellaneous ****************/
#define VBAT_ADC                 	(&hadc1)
#define ADC_TO_VBAT_MULTIPLIER   	(3300 * 5 / 4096)
#define ADC_TO_VBAT_OFFSET		 	220
#define BATTERY_INDICATOR_ROW    	0
#define BATTERY_INDICATOR_COL    	15
#define BATTERY_INDICATOR_PERIOD 	10


#endif /* INC_CONFIG_H_ */
