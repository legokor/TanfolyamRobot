/*
 * control.c
 *
 *  Created on: Jan 28, 2025
 *      Author: dkiovics
 */

#include "control.h"
#include "config.h"
#include "robotInstance.h"
#include "main.h"
#include "telemetry.h"
#include "lcd.h"
#include "status_indicator.h"
#include "robotAbstraction-api.h"
#include "application.h"


main_Telemetry main_telemetry;
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

	HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_SET);
	lcdInit(LCD_RST_GPIO_Port, LCD_RST_Pin, LCD_EN_GPIO_Port, LCD_EN_Pin,
			LCD_D4_GPIO_Port, LCD_D4_Pin, LCD_D5_GPIO_Port, LCD_D5_Pin,
			LCD_D6_GPIO_Port, LCD_D6_Pin, LCD_D7_GPIO_Port, LCD_D7_Pin,
			2, 16);

	uart_init(&main_robotInstance.usbUart, USB_UART, USB_UART_IR, USB_UART_DMA_IR, 800, 800, "\r");
	uart_init(&main_robotInstance.espUart, ESP_UART, ESP_UART_IR, ESP_UART_DMA_IR, 800, 800, "\r\x01\x02\x03");

	#if US_SENSOR
	usInit(&main_robotInstance.us, US_TRIG_GPIO_Port, US_TRIG_Pin,
		  RANGE_COLOR_ESP_TIMER, US_TIMER_FREQUENCY_HZ,
		  RANGE_COLOR_ESP_TIMER, US_TIMER_FREQUENCY_HZ);
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, US_RISING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, US_FALLING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_OC_Start_IT(RANGE_COLOR_ESP_TIMER, US_ASYNC_CHANNEL) != HAL_OK)
		FAIL;
	#elif IR_SENSOR
	irInit(&main_robotInstance.ir, RANGE_COLOR_ESP_TIMER, IR_TIMER_FREQUENCY_HZ);
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, IR_RISING_CHANNEL) != HAL_OK)
		FAIL;
	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, IR_FALLING_CHANNEL) != HAL_OK)
		FAIL;
	#endif

	colorSensorInit(&main_robotInstance.colorSensor,
					COLOR_S0_GPIO_Port, COLOR_S0_Pin, COLOR_S1_GPIO_Port, COLOR_S1_Pin,
					COLOR_S2_GPIO_Port, COLOR_S2_Pin, COLOR_S3_GPIO_Port, COLOR_S3_Pin,
					16);

	if (HAL_TIM_IC_Start_IT(RANGE_COLOR_ESP_TIMER, COLOR_CHANNEL) != HAL_OK)
		FAIL;

	if (HAL_TIM_Base_Start_IT(VBAT_ADC_TIMER) != HAL_OK)
		FAIL;
	if (HAL_ADC_Start_IT(VBAT_ADC) != HAL_OK)
		FAIL;

	encoderInit(&main_robotInstance.rightEncoder, ENC1_A_GPIO_Port, ENC1_A_Pin, ENC1_B_GPIO_Port, ENC1_B_Pin,
				MOTOR1_ENCODER_RESOLUTION, MOTOR1_REVERSED,
				LCD_IMU_ENCODER_TIMER, MOTOR_ENCODER_MAX_SPEED_CPS);
	encoderInit(&main_robotInstance.leftEncoder, ENC2_A_GPIO_Port, ENC2_A_Pin, ENC2_B_GPIO_Port, ENC2_B_Pin,
				MOTOR2_ENCODER_RESOLUTION, MOTOR2_REVERSED,
				LCD_IMU_ENCODER_TIMER, MOTOR_ENCODER_MAX_SPEED_CPS);

	{
		Pwm pwm1; pwmInit(&pwm1, MOTOR12_PWM1_TIMER, MOTOR1_PWM1_TIMER_CHANNEL, MOTOR1_PWM1_TIMER_PERIOD, MOTOR1_PWM1_OUTPUT_TYPE);
		Pwm pwm2; pwmInit(&pwm2, MOTOR12_PWM2_TIMER, MOTOR1_PWM2_TIMER_CHANNEL, MOTOR1_PWM2_TIMER_PERIOD, MOTOR1_PWM2_OUTPUT_TYPE);
		motorInit(&main_robotInstance.rightMotor, pwm1, pwm2, MOTOR1_REVERSED);
		speedControlInit(&main_robotInstance.rightSpeedCtrl, &main_robotInstance.rightMotor, &main_robotInstance.rightEncoder);
	}
	{
		Pwm pwm1; pwmInit(&pwm1, MOTOR12_PWM1_TIMER, MOTOR2_PWM1_TIMER_CHANNEL, MOTOR2_PWM1_TIMER_PERIOD, MOTOR2_PWM1_OUTPUT_TYPE);
		Pwm pwm2; pwmInit(&pwm2, MOTOR12_PWM2_TIMER, MOTOR2_PWM2_TIMER_CHANNEL, MOTOR2_PWM2_TIMER_PERIOD, MOTOR2_PWM2_OUTPUT_TYPE);
		motorInit(&main_robotInstance.leftMotor, pwm1, pwm2, MOTOR2_REVERSED);
		speedControlInit(&main_robotInstance.leftSpeedCtrl, &main_robotInstance.leftMotor, &main_robotInstance.leftEncoder);
	}

	HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_SET);

	{
		Pwm pwm; pwmInit(&pwm, SERVO_TIMER, SERVO_CHANNEL, SERVO_PWM_PERIOD, PwmOutput_P);
		servoInit(&main_robotInstance.servo, pwm, SERVO_START_POS, SERVO_END_POS, SERVO_INIT_POS);
	}

	mpu9250_init(&main_robotInstance.imu, IMU_I2C, 0x68, 0x0C, IMU_I2C_IT_IR);
	mpu9250_setDefaultSettings(&main_robotInstance.imu);

	main_telemetryInit(&main_telemetry);

	main_robotInstance.initCplt = 1;

	statusIndicatorInit();
	lcdPuts(0, 4, "LEGO");
	lcdPuts(1, 8, "K\xefR");

	HAL_Delay(800);

	if(IMU_GYRO_OFFSET_EN){
		lcdClear();
		lcdPuts(0, 1, "Gyro calib..");
		lcdPuts(1, 3, "DO NOT MOVE!");
		mpu9250_calculateGyroOffset(&main_robotInstance.imu);
		mpu9250_enableGyroOffsetSubtraction(&main_robotInstance.imu, 1);
	}

	main_robotInstance.espReady = 1;

	main_robotInstance.orientation.pitch = 0;
	main_robotInstance.orientation.roll = 0;

	lcdClear();
	statusIndicatorEnable();

	main_sendConfigToEsp(&main_robotInstance);
}


void main_runRobot(void)
{
	lcdPrintf(0, 0, "Press button\nto start");
	while (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET);
	lcdClear();

	int retVal = application();

	HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_RESET);
	speedControlSetSpeed(&main_robotInstance.rightSpeedCtrl, 0);
	speedControlSetSpeed(&main_robotInstance.leftSpeedCtrl, 0);
	lcdClear();
	lcdPrintf(0, 0, "application");
	lcdPrintf(1, 0, "returned %d", retVal);
}

