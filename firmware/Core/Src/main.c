/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "robotcontrol.h"
#include "lcd.h"
#include "soft-pwm.h"
#include "battery_indicator.h"
#include "color_sensor.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "servo.h"
#include "motor.h"
#include "uart.h"
#include "speed_control.h"
#include "application.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_TIMER              (&htim1)
#define LCD_BL_TIMER           (&htim3)
#define LCD_BL_CHANNEL         TIM_CHANNEL_1
#define LCD_BL_ACTIVE_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_1
#define LCD_BL_TIMER_PERIOD    5333
#define LCD_BL_PERCENT         50
#define LCD_BL_PWM_INVERTED     0

#define USB_UART (&huart1)
#define USB_UART_IR USART1_IRQn
#define USB_UART_DMA_IR DMA1_Channel4_IRQn

#define ESP_UART (&huart3)
#define ESP_UART_IR USART3_IRQn
#define ESP_UART_DMA_IR DMA1_Channel2_IRQn

#define VBAT_ADC                 (&hadc1)
#define VBAT_ADC_TIMER           (&htim4)
#define ADC_TO_VBAT_MULTIPLIER   (3300 * 5 / 4096)
#define ADC_TO_VBAT_OFFSET		 220
#define BATTERY_INDICATOR_ROW    0
#define BATTERY_INDICATOR_COL    15
#define BATTERY_INDICATOR_PERIOD 10   // about 300 ms

#define SERVO_TIMER          (&htim2)
#define SERVO_CHANNEL        TIM_CHANNEL_3
#define SERVO_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_3
#define SERVO_PWM_INVERTED         1
#define SERVO_TIMER_PERIOD   0x10000
#define SERVO_PWM_PERIOD       20000
#define SERVO_START_POS         680           // TODO: calibrate endpoints
#define SERVO_END_POS           2640
#define SERVO_INIT_POS 			0

#define US_COLOR_ESP_TX_CAPTURE_TIMER (&htim4)
#define US_TIMER_FREQUENCY_HZ      (2*1000*1000)
#define US_RISING_CHANNEL          TIM_CHANNEL_1
#define US_RISING_ACTIVE_CHANNEL   HAL_TIM_ACTIVE_CHANNEL_1
#define US_FALLING_CHANNEL         TIM_CHANNEL_2
#define US_FALLING_ACTIVE_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_2
#define COLOR_CHANNEL              TIM_CHANNEL_3
#define COLOR_ACTIVE_CHANNEL       HAL_TIM_ACTIVE_CHANNEL_3
#define US_ASYNC_ACTIVE_CHANNEL    HAL_TIM_ACTIVE_CHANNEL_4
#define US_ASYNC_CHANNEL    	   TIM_CHANNEL_4

#define MOTOR1_PWM1_TIMER         (&htim1)
#define MOTOR1_PWM1_TIMER_CHANNEL TIM_CHANNEL_1
#define MOTOR1_PWM1_TIMER_PERIOD  4000
#define MOTOR1_PWM1_OUTPUT_TYPE   PwmOutput_N
#define MOTOR1_PWM2_TIMER         (&htim3)
#define MOTOR1_PWM2_TIMER_CHANNEL TIM_CHANNEL_3
#define MOTOR1_PWM2_TIMER_PERIOD  4000
#define MOTOR1_PWM2_OUTPUT_TYPE   PwmOutput_P
#define MOTOR1_REVERSED           1

#define MOTOR2_PWM1_TIMER         (&htim1)
#define MOTOR2_PWM1_TIMER_CHANNEL TIM_CHANNEL_2
#define MOTOR2_PWM1_TIMER_PERIOD  4000
#define MOTOR2_PWM1_OUTPUT_TYPE   PwmOutput_N
#define MOTOR2_PWM2_TIMER         (&htim3)
#define MOTOR2_PWM2_TIMER_CHANNEL TIM_CHANNEL_4
#define MOTOR2_PWM2_TIMER_PERIOD  4000
#define MOTOR2_PWM2_OUTPUT_TYPE   PwmOutput_P
#define MOTOR2_REVERSED           0

#define MOTOR_CONTROL_TIMER       (&htim3)
#define MOTOR_CONTROL_PRESCALE    16                // The speed controller should run on every n-th interrupt

#define MOTOR1_ENCODER_RESOLUTION EncoderResolution_1
#define MOTOR2_ENCODER_RESOLUTION EncoderResolution_1
#define MOTOR_ENCODER_TIMER       (&htim1)
#define MOTOR_ENCODER_MAX_SPEED_CPS 3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define FAIL fail(__FILE__, __LINE__)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
const uint8_t lcdRows = 2;
const uint8_t lcdCols = 16;

volatile Encoder encoder1;
volatile Encoder encoder2;

volatile UltraSonic us;
volatile ColorSensor colorSensor;

Servo* servo;

volatile Motor* motor1;
volatile Motor* motor2;
volatile SpeedControl* speedControl1;
volatile SpeedControl* speedControl2;

volatile Uart uart1;
volatile Uart uart3;

volatile uint16_t batteryVoltage = 0;
volatile uint8_t batteryAdcBusy = 0;

volatile uint8_t pgmReady = 0;
volatile uint8_t espReady = 0;

volatile SoftPwm* lcdBacklightPwm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t adcTimerItCount = 0;
    static uint32_t telemetryItCounter = 0;

    if(!pgmReady)
    	return;

    if(htim == MOTOR_ENCODER_TIMER){
    	encoderTimerOverflowHandler(&encoder1);
    	encoderTimerOverflowHandler(&encoder2);
    }
    if (htim == LCD_TIMER) {
        lcdHandler();
    }
    if (htim == VBAT_ADC_TIMER) {
        adcTimerItCount++;

        if (adcTimerItCount >= BATTERY_INDICATOR_PERIOD) {
            adcTimerItCount = 0;
            batteryIndicatorDisplay(BATTERY_INDICATOR_ROW, BATTERY_INDICATOR_COL, batteryVoltage);
        }

        if (!batteryAdcBusy) {
            batteryAdcBusy = 1;
            HAL_ADC_Start_IT(VBAT_ADC);
        }
    }

    if(htim == US_COLOR_ESP_TX_CAPTURE_TIMER){
    	usStartMeasurementPulseAsync(&us);
    	telemetryItCounter++;
    	if(telemetryItCounter == 4){
    		telemetryItCounter = 0;
    		uart_sendDataToEsp(&uart3, &colorSensor, speedControl1, speedControl2, servo, &us);
    	}
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if(!pgmReady)
	    return;

    static uint8_t scPsc = 0;
    if (htim == MOTOR_CONTROL_TIMER) {        // TODO: maybe use the period interrupt insted of the pulse finished interrupt
        scPsc++;
        if (scPsc == MOTOR_CONTROL_PRESCALE) {
            scPsc = 0;
            if (speedControl1 != NULL) {
                speedControlHandler(speedControl1);
            }
            if (speedControl2 != NULL) {
                speedControlHandler(speedControl2);
            }
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(!pgmReady)
	    return;

    if (hadc == VBAT_ADC) {
        uint16_t adcVal = HAL_ADC_GetValue(VBAT_ADC);
        batteryVoltage =  adcVal * ADC_TO_VBAT_MULTIPLIER + ADC_TO_VBAT_OFFSET;
        batteryAdcBusy = 0;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(!pgmReady)
	    return;

    if (htim == US_COLOR_ESP_TX_CAPTURE_TIMER) {
        switch (htim->Channel) {
            case US_RISING_ACTIVE_CHANNEL : {
                uint16_t captureVal = HAL_TIM_ReadCapturedValue(htim, US_RISING_CHANNEL);
                usHandlerRisingCapture(&us, captureVal);
                break;
            }
            case US_FALLING_ACTIVE_CHANNEL : {
                uint16_t captureVal = HAL_TIM_ReadCapturedValue(htim, US_FALLING_CHANNEL);
                usHandlerFallingCapture(&us, captureVal);
                break;
            }
            case COLOR_ACTIVE_CHANNEL : {
                uint16_t captureVal = HAL_TIM_ReadCapturedValue(htim, COLOR_CHANNEL);
                colorSensorCaptureHandler(&colorSensor, captureVal);
                break;
            }
            default: break;  // only needed to suppress unhandled enum value warning
        }
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if(!pgmReady)
	    return;

    if(htim == US_COLOR_ESP_TX_CAPTURE_TIMER && htim->Channel == US_ASYNC_ACTIVE_CHANNEL){
    	usHandleCompareAsync(&us);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	uart_handleReceiveCplt(&uart1, USB_UART, pgmReady);
	uart_handleReceiveCplt(&uart3, ESP_UART, pgmReady && espReady);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(!pgmReady)
	    return;

	if (huart == USB_UART) {
		uart_handleTransmitCplt(&uart1, huart);
	}
	if (huart == ESP_UART) {
		uart_handleTransmitCplt(&uart3, huart);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(!pgmReady)
	    return;

    switch (GPIO_Pin) {
        case ENC1_A_Pin: encoderHandlerA(&encoder1); break;
        //case ENC1_B_Pin: encoderHandlerB(&encoder1); break;
        case ENC2_A_Pin: encoderHandlerA(&encoder2); break;
        //case ENC2_B_Pin: encoderHandlerB(&encoder2); break;
    }
}

/**
 * Print error message on LCD and UART, then wait indefinitely for UART DFU request
 * @note If both LCD and UART init failed, we are SOL
 * @param file file name (__FILE__)
 * @param line file line (__LINE__)
 */
void fail(char *file, uint16_t line) {
    char msg[512];
    uint16_t msgLen = sprintf(msg, "ERROR %s:%d", file, line);

    lcdPuts(0, 0, msg);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, msgLen, 0xffff);

    while (1) {

    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(LCD_TIMER) != HAL_OK) {
      FAIL;
  }

  // TODO
/*
  lcdBacklightPwm = softPwmCreate(LCD_BL_TIMER, LCD_BL_CHANNEL, LCD_BL_TIMER_PERIOD,
                                  LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin,
                                  LCD_BL_TIMER_PERIOD, LCD_BL_PWM_INVERTED);
  if (lcdBacklightPwm == NULL) {
      FAIL;
  }
  softPwmSetDutyCylePercent(lcdBacklightPwm, LCD_BL_PERCENT);
*/
  HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_SET);

  lcdInit(LCD_RST_GPIO_Port, LCD_RST_Pin, LCD_EN_GPIO_Port, LCD_EN_Pin,
          LCD_D4_GPIO_Port, LCD_D4_Pin, LCD_D5_GPIO_Port, LCD_D5_Pin,
          LCD_D6_GPIO_Port, LCD_D6_Pin, LCD_D7_GPIO_Port, LCD_D7_Pin,
          lcdRows, lcdCols);

  uart_init(&uart1, USB_UART, USB_UART_IR, USB_UART_DMA_IR, 500, 500);
  uart_init(&uart3, ESP_UART, ESP_UART_IR, ESP_UART_DMA_IR, 500, 500);

  batteryIndicatorInit();

  lcdPuts(0, 4, "LEGO");
  lcdPuts(1, 8, "K\xefR");

  usInit(&us, US_TRIG_GPIO_Port, US_TRIG_Pin,
         US_COLOR_ESP_TX_CAPTURE_TIMER, US_TIMER_FREQUENCY_HZ,
         US_COLOR_ESP_TX_CAPTURE_TIMER, US_TIMER_FREQUENCY_HZ);

  if (HAL_TIM_IC_Start_IT(US_COLOR_ESP_TX_CAPTURE_TIMER, US_RISING_CHANNEL) != HAL_OK) {
      FAIL;
  }
  if (HAL_TIM_IC_Start_IT(US_COLOR_ESP_TX_CAPTURE_TIMER, US_FALLING_CHANNEL) != HAL_OK) {
      FAIL;
  }
  if (HAL_TIM_OC_Start_IT(US_COLOR_ESP_TX_CAPTURE_TIMER, US_ASYNC_CHANNEL) != HAL_OK) {
	  FAIL;
  }

  colorSensorInit(&colorSensor,
                  COLOR_S0_GPIO_Port, COLOR_S0_Pin, COLOR_S1_GPIO_Port, COLOR_S1_Pin,
                  COLOR_S2_GPIO_Port, COLOR_S2_Pin, COLOR_S3_GPIO_Port, COLOR_S3_Pin,
                  16);

  if (HAL_TIM_IC_Start_IT(US_COLOR_ESP_TX_CAPTURE_TIMER, COLOR_CHANNEL) != HAL_OK) {
      FAIL;
  }

  if (HAL_TIM_Base_Start_IT(VBAT_ADC_TIMER) != HAL_OK) {
      FAIL;
  }
  if (HAL_ADC_Start_IT(VBAT_ADC) != HAL_OK) {
      FAIL;
  }

  encoderInit(&encoder1, ENC1_A_GPIO_Port, ENC1_A_Pin, ENC1_B_GPIO_Port, ENC1_B_Pin,
                         MOTOR1_ENCODER_RESOLUTION, MOTOR1_REVERSED,
                         MOTOR_ENCODER_TIMER, MOTOR_ENCODER_MAX_SPEED_CPS);
  encoderInit(&encoder2, ENC2_A_GPIO_Port, ENC2_A_Pin, ENC2_B_GPIO_Port, ENC2_B_Pin,
                         MOTOR2_ENCODER_RESOLUTION, MOTOR2_REVERSED,
                         MOTOR_ENCODER_TIMER, MOTOR_ENCODER_MAX_SPEED_CPS);

  motor1 = motorCreate(MOTOR1_PWM1_TIMER, MOTOR1_PWM1_TIMER_CHANNEL,
                       MOTOR1_PWM1_TIMER_PERIOD, MOTOR1_PWM1_OUTPUT_TYPE,
                       MOTOR1_PWM2_TIMER, MOTOR1_PWM2_TIMER_CHANNEL,
                       MOTOR1_PWM2_TIMER_PERIOD, MOTOR1_PWM2_OUTPUT_TYPE,
                       MOTOR1_REVERSED);
  if (motor1 == NULL) {
      FAIL;
  }
  motor2 = motorCreate(MOTOR2_PWM1_TIMER, MOTOR2_PWM1_TIMER_CHANNEL,
                       MOTOR2_PWM1_TIMER_PERIOD, MOTOR2_PWM1_OUTPUT_TYPE,
                       MOTOR2_PWM2_TIMER, MOTOR2_PWM2_TIMER_CHANNEL,
                       MOTOR2_PWM2_TIMER_PERIOD, MOTOR2_PWM2_OUTPUT_TYPE,
                       MOTOR2_REVERSED);
  if (motor2 == NULL) {
      FAIL;
  }

  speedControl1 = speedControlCreate(motor1, &encoder1);
  if (speedControl1 == NULL) {
      FAIL;
  }

  speedControl2 = speedControlCreate(motor2, &encoder2);
  if (speedControl2 == NULL) {
      FAIL;
  }

  HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_SET);

  servo = servoCreate(SERVO_TIMER, SERVO_CHANNEL, SERVO_PWM_PERIOD, PwmOutput_P, SERVO_START_POS, SERVO_END_POS, SERVO_INIT_POS);

  robotControlInit(servo, &us, &colorSensor, speedControl2, speedControl1, &encoder2, &encoder1, &uart1, &uart3);

  pgmReady = 1;
  HAL_Delay(1200);
  espReady = 1;
  lcdClear();
  batteryIndicatorEnable();

  uart_sendConfigToEsp(&uart3, WIFI_SSID, WIFI_PASSWORD, SERVER_IP);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  lcdPrintf(0, 0, "Press button\nto start");
  while (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET);
  lcdClear();

  // Start user application
  int retVal = application();

  // If application returns, stop the motors, and print the returned value
  HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port, MOTOR_SLEEPN_Pin, GPIO_PIN_RESET);
  speedControlSetSpeed(speedControl1, 0);
  speedControlSetSpeed(speedControl2, 0);
  lcdClear();
  lcdPrintf(0, 0, "application");
  lcdPrintf(1, 0, "returned %d", retVal);

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
                          |LCD_EN_Pin|LCD_RST_Pin|MOTOR_SLEEPN_Pin|COLOR_S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COLOR_S1_GPIO_Port, COLOR_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, COLOR_S3_Pin|COLOR_S2_Pin|US_TRIG_Pin|LCD_BACKLIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_EN_Pin LCD_RST_Pin MOTOR_SLEEPN_Pin COLOR_S0_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin
                          |LCD_EN_Pin|LCD_RST_Pin|MOTOR_SLEEPN_Pin|COLOR_S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_A_Pin ENC2_A_Pin */
  GPIO_InitStruct.Pin = ENC1_A_Pin|ENC2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_B_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC1_B_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_FAULTN_Pin */
  GPIO_InitStruct.Pin = MOTOR_FAULTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MOTOR_FAULTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PWR_DET_Pin */
  GPIO_InitStruct.Pin = USB_PWR_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USB_PWR_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLOR_S1_Pin */
  GPIO_InitStruct.Pin = COLOR_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COLOR_S1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLOR_S3_Pin COLOR_S2_Pin LCD_BACKLIGHT_Pin */
  GPIO_InitStruct.Pin = COLOR_S3_Pin|COLOR_S2_Pin|LCD_BACKLIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : US_TRIG_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
