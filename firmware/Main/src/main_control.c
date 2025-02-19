/*
 * main_control.c
 *
 *  Created on: Jan 28, 2025
 *      Author: dkiovics
 */

#include "main_interface.h"
#include "main_config.h"
#include "main.h"
#include "tel_interface.h"
#include "lcd_interface.h"
#include "api_robotAbstraction.h"
#include "app_interface.h"


main_RobotInstance main_robotInstance = { .initCplt = 0, .espReady = 0 };

#define FAIL fail(__FILE__, __LINE__)

void fail(const char *file, uint16_t line)
{
    while (1) { }
}


void main_initRobot(void)
{
	if (HAL_TIM_Base_Start_IT(LCD_IMU_ENCODER_TIMER) != HAL_OK)
		FAIL;
	if (HAL_TIM_Base_Start_IT(SERVO_DELAYUS_TIMER) != HAL_OK)
		FAIL;

	HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_SET);
	lcd_init(LCD_RST_GPIO_Port, LCD_RST_Pin, LCD_EN_GPIO_Port, LCD_EN_Pin,
			LCD_D4_GPIO_Port, LCD_D4_Pin, LCD_D5_GPIO_Port, LCD_D5_Pin,
			LCD_D6_GPIO_Port, LCD_D6_Pin, LCD_D7_GPIO_Port, LCD_D7_Pin,
			2, 16);

	txt_init(&main_robotInstance.usbUart, USB_UART, USB_UART_IR, USB_UART_DMA_IR, 500, 500, "\r");
	txt_init(&main_robotInstance.espUart, ESP_UART, ESP_UART_IR, ESP_UART_DMA_IR, 500, 500, "\r\x01\x02\x03");

	#if US_SENSOR
	us_init(&main_robotInstance.us, US_TRIG_GPIO_Port, US_TRIG_Pin,
		  RANGE_COLOR_ESP_TIMER, US_TIMER_FREQUENCY_HZ,
		  RANGE_COLOR_ESP_TIMER, US_TIMER_FREQUENCY_HZ);
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, US_RISING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, US_FALLING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_OC_Start_IT(RANGE_COLOR_ESP_TIMER, US_ASYNC_CHANNEL) != HAL_OK)
		FAIL;
	#elif IR_SENSOR
	ir_init(&main_robotInstance.ir, RANGE_COLOR_ESP_TIMER, IR_TIMER_FREQUENCY_HZ);
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, IR_RISING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, IR_FALLING_CHANNEL) != HAL_OK)
		FAIL;
	#endif

	clr_init(&main_robotInstance.colorSensor,
					COLOR_S0_GPIO_Port, COLOR_S0_Pin, COLOR_S1_GPIO_Port, COLOR_S1_Pin,
					COLOR_S2_GPIO_Port, COLOR_S2_Pin, COLOR_S3_GPIO_Port, COLOR_S3_Pin,
					16);

	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, COLOR_CHANNEL) != HAL_OK)
		FAIL;

	if (HAL_TIM_Base_Start_IT(VBAT_ADC_TIMER) != HAL_OK)
		FAIL;
	if (HAL_ADC_Start_IT(VBAT_ADC) != HAL_OK)
		FAIL;

	enc_init(&main_robotInstance.rightEncoder, ENC1_A_GPIO_Port, ENC1_A_Pin, ENC1_B_GPIO_Port, ENC1_B_Pin,
				MOTOR1_ENCODER_RESOLUTION, MOTOR1_REVERSED,
				LCD_IMU_ENCODER_TIMER, MOTOR_ENCODER_TIMER_FREQ, MOTOR_ENCODER_MAX_SPEED_CPS);
	enc_init(&main_robotInstance.leftEncoder, ENC2_A_GPIO_Port, ENC2_A_Pin, ENC2_B_GPIO_Port, ENC2_B_Pin,
				MOTOR2_ENCODER_RESOLUTION, MOTOR2_REVERSED,
				LCD_IMU_ENCODER_TIMER, MOTOR_ENCODER_TIMER_FREQ, MOTOR_ENCODER_MAX_SPEED_CPS);

	{
		pwm_Pwm pwm1; pwm_init(&pwm1, MOTOR12_PWM1_TIMER, MOTOR1_PWM1_TIMER_CHANNEL, MOTOR1_PWM1_TIMER_PERIOD, MOTOR1_PWM1_OUTPUT_TYPE);
		pwm_Pwm pwm2; pwm_init(&pwm2, MOTOR12_PWM2_TIMER, MOTOR1_PWM2_TIMER_CHANNEL, MOTOR1_PWM2_TIMER_PERIOD, MOTOR1_PWM2_OUTPUT_TYPE);
		drv_motorInit(&main_robotInstance.rightMotor, pwm1, pwm2, MOTOR1_REVERSED);
		drv_speedControlInit(&main_robotInstance.rightSpeedCtrl, &main_robotInstance.rightMotor, &main_robotInstance.rightEncoder);
	}
	{
		pwm_Pwm pwm1; pwm_init(&pwm1, MOTOR12_PWM1_TIMER, MOTOR2_PWM1_TIMER_CHANNEL, MOTOR2_PWM1_TIMER_PERIOD, MOTOR2_PWM1_OUTPUT_TYPE);
		pwm_Pwm pwm2; pwm_init(&pwm2, MOTOR12_PWM2_TIMER, MOTOR2_PWM2_TIMER_CHANNEL, MOTOR2_PWM2_TIMER_PERIOD, MOTOR2_PWM2_OUTPUT_TYPE);
		drv_motorInit(&main_robotInstance.leftMotor, pwm1, pwm2, MOTOR2_REVERSED);
		drv_speedControlInit(&main_robotInstance.leftSpeedCtrl, &main_robotInstance.leftMotor, &main_robotInstance.leftEncoder);
	}

	HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_SET);

	{
		pwm_Pwm pwm; pwm_init(&pwm, SERVO_DELAYUS_TIMER, SERVO_CHANNEL, SERVO_PWM_PERIOD, PwmOutput_P);
		srv_init(&main_robotInstance.servo, pwm, SERVO_START_POS, SERVO_END_POS, SERVO_INIT_POS);
	}

	mpu_init(&main_robotInstance.imu, IMU_I2C, 0x68, 0x0C, IMU_I2C_IT_IR);
	mpu_setDefaultSettings(&main_robotInstance.imu);

	main_robotInstance.initCplt = 1;

	lcd_puts(0, 4, "LEGO");
	lcd_puts(1, 8, "K\xefR");

	HAL_Delay(800);

	if(IMU_GYRO_OFFSET_EN){
		lcd_clear();
		lcd_puts(0, 1, "Gyro calib..");
		lcd_puts(1, 3, "DO NOT MOVE!");
		mpu_calculateGyroOffset(&main_robotInstance.imu);
		mpu_enableGyroOffsetSubtraction(&main_robotInstance.imu, 1);
	}

	main_robotInstance.espReady = 1;

	main_robotInstance.orientation.pitch = 0;
	main_robotInstance.orientation.roll = 0;

	lcd_clear();
	lcd_enableStatus();

	if (!tel_initEsp())
	{
		lcd_clear();
		lcd_puts(0, 0, "ESP init failed");
		while (1) { }
	}
}


void main_runRobot(void)
{
	lcdPrintf(0, 0, "Press button\nto start");
	while (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET);
	lcd_clear();

	int retVal = app_main();

	HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_RESET);
	drv_speedControlSetSpeed(&main_robotInstance.rightSpeedCtrl, 0);
	drv_speedControlSetSpeed(&main_robotInstance.leftSpeedCtrl, 0);
	lcd_clear();
	lcdPrintf(0, 0, "application");
	lcdPrintf(1, 0, "returned %d", retVal);
}

