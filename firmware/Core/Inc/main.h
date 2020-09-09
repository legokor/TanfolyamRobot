/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_D7_Pin GPIO_PIN_13
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_15
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_0
#define LCD_D4_GPIO_Port GPIOC
#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOC
#define TIM2_CH1_ENC1A_Pin GPIO_PIN_0
#define TIM2_CH1_ENC1A_GPIO_Port GPIOA
#define TIM2_CH1_ENC1B_Pin GPIO_PIN_1
#define TIM2_CH1_ENC1B_GPIO_Port GPIOA
#define TIM3_CH1_ENC2A_Pin GPIO_PIN_6
#define TIM3_CH1_ENC2A_GPIO_Port GPIOA
#define TIM3_CH2_ENC2B_Pin GPIO_PIN_7
#define TIM3_CH2_ENC2B_GPIO_Port GPIOA
#define MOTOR_FAULTN_Pin GPIO_PIN_4
#define MOTOR_FAULTN_GPIO_Port GPIOC
#define MOTOR_SLEEPN_Pin GPIO_PIN_5
#define MOTOR_SLEEPN_GPIO_Port GPIOC
#define TIM3_CH3_MOTOR1_PWM2_Pin GPIO_PIN_0
#define TIM3_CH3_MOTOR1_PWM2_GPIO_Port GPIOB
#define TIM3_CH4_MOTOR2_PWM2_Pin GPIO_PIN_1
#define TIM3_CH4_MOTOR2_PWM2_GPIO_Port GPIOB
#define USB_PWR_DET_Pin GPIO_PIN_12
#define USB_PWR_DET_GPIO_Port GPIOB
#define TIM1_CH1N_MOTOR2_PWM1_Pin GPIO_PIN_13
#define TIM1_CH1N_MOTOR2_PWM1_GPIO_Port GPIOB
#define TIM1_CH2N_MOTOR1_PWM1_Pin GPIO_PIN_14
#define TIM1_CH2N_MOTOR1_PWM1_GPIO_Port GPIOB
#define TIM1_CH3N_SERVO_PWM_Pin GPIO_PIN_15
#define TIM1_CH3N_SERVO_PWM_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOA
#define COLOR_S0_Pin GPIO_PIN_12
#define COLOR_S0_GPIO_Port GPIOC
#define COLOR_S1_Pin GPIO_PIN_2
#define COLOR_S1_GPIO_Port GPIOD
#define COLOR_S3_Pin GPIO_PIN_4
#define COLOR_S3_GPIO_Port GPIOB
#define COLOR_S2_Pin GPIO_PIN_5
#define COLOR_S2_GPIO_Port GPIOB
#define US_ECHO_Pin GPIO_PIN_6
#define US_ECHO_GPIO_Port GPIOB
#define US_TRIG_Pin GPIO_PIN_7
#define US_TRIG_GPIO_Port GPIOB
#define TIM4_CH3_COLOR_Pin GPIO_PIN_8
#define TIM4_CH3_COLOR_GPIO_Port GPIOB
#define LCD_BACKLIGHT_Pin GPIO_PIN_9
#define LCD_BACKLIGHT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
