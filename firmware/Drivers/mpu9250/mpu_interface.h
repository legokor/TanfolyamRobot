/*
 * mpu_interface.h
 *
 *  Created on: Feb 1, 2025
 *      Author: dkiovics
 */

#ifndef MPU9250_MPU_INTERFACE_H_
#define MPU9250_MPU_INTERFACE_H_

#define ASYNC_IMU

#include "stm32f1xx_hal.h"

/**
 * @brief MPU9250 IMU and compass IC I2C driver for the STM32 microcontroller.
 * The implementation heavily relies on the HAL STM32 drivers.
 *
 */
typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint8_t imuAddress;
	uint8_t magAddress;

	float magCoeff_x, magCoeff_y, magCoeff_z;
	float accSensitivity;
	float gyroSensitivity;

	float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
	uint8_t useGyroOffsets;

	volatile uint8_t imuData[14];
	volatile uint8_t magData[7];
	volatile uint8_t initComplete;

#ifdef ASYNC_IMU
	volatile uint8_t imuBuffer[14];
	volatile uint8_t magBuffer[7];
	volatile uint8_t isReadingImu;
	volatile uint8_t newData;

	IRQn_Type readIr;

	volatile uint8_t readEnabled;
	volatile uint8_t readInProgress;
#endif
} mpu_Mpu9250;

typedef struct {
	float x, y, z;
} mpu_Vec3;

typedef struct {
	float pitch;
	float roll;
} mpu_Orientation;


#ifdef ASYNC_IMU
void mpu_init(mpu_Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t imuAddress, uint8_t magAddress, IRQn_Type readIr);
#else
void mpu_init(mpu_Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t imuAddress, uint8_t magAddress);
#endif

uint8_t mpu_detectImu(mpu_Mpu9250* mpu);

uint8_t mpu_detectMagnetometer(mpu_Mpu9250* mpu);

void mpu_setDefaultSettings(mpu_Mpu9250* mpu);

void mpu_calculateGyroOffset(mpu_Mpu9250* mpu);

void mpu_enableGyroOffsetSubtraction(mpu_Mpu9250* mpu, uint8_t enabled);

void mpu_setSampleRateDivider(mpu_Mpu9250* mpu, uint8_t divider);

void mpu_enableAccDLPF(mpu_Mpu9250* mpu, uint8_t enable);

void mpu_enableGyroAndTempDLPF(mpu_Mpu9250* mpu, uint8_t enable);

void mpu_setAccDLPF(mpu_Mpu9250* mpu, uint8_t value);

void mpu_setGyroAndTempDLPF(mpu_Mpu9250* mpu, uint8_t value);

void mpu_setGyroSensitivity(mpu_Mpu9250* mpu, uint8_t sensitivity);

void mpu_setAccSensitivity(mpu_Mpu9250* mpu, uint8_t sensitivity);

#ifdef ASYNC_IMU
void mpu_i2cReceiveCpltCallback(mpu_Mpu9250* mpu);

void mpu_timPeriodEllapsedCallback(mpu_Mpu9250* mpu);

uint8_t mpu_newDataAvailable(mpu_Mpu9250* mpu);
#endif

mpu_Vec3 mpu_readGyroData(mpu_Mpu9250* mpu);

mpu_Vec3 mpu_readAccData(mpu_Mpu9250* mpu);

float mpu_readTempData(mpu_Mpu9250* mpu);

mpu_Vec3 mpu_readMagData(mpu_Mpu9250* mpu);

void mpu_updateOrientation(mpu_Mpu9250* imu, volatile mpu_Orientation* orientation, float dt);

#endif /* MPU9250_MPU_INTERFACE_H_ */
