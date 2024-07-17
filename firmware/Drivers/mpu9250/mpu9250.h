/*
 * mpu9250.h
 *
 *  Created on: May 20, 2024
 *      Author: dkiovics
 */

#ifndef MPU9250_MPU9250_H_
#define MPU9250_MPU9250_H_

#define ASYNC_IMU

#include "stm32f1xx_hal.h"
#include "vec3.h"

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
} Mpu9250;



#ifdef ASYNC_IMU
/**
 * @brief Constructs the Mpu9250 class for async data retrieval using DMA.
 *
 * @param hi2c - the I2C HAL object pointer. The global interrupts must be enabled in the CubeMX.
 * @param imuAddress - the IMU IC I2C address (in reality this is the MPU9250 IC, but the IC's package also contains
 * an AK8963 magnetometer IC).
 * @param magAddress - the magnetometer IC I2C address (generally it is 0x0C).
 * @param dmaIr - the IRQn_Type handle of the I2C event interrupt
 */
void mpu9250_init(Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t imuAddress, uint8_t magAddress, IRQn_Type readIr);
#else
/**
 * @brief Constructs the Mpu9250 class for sync (blocking) data retrieval.
 *
 * @param hi2c - the I2C HAL object pointer.
 * @param imuAddress - the IMU IC I2C address (in reality this is the MPU9250 IC, but the IC's package also contains
 * an AK8963 magnetometer IC).
 * @param magAddress - the magnetometer IC I2C address (generally it is 0x0C).
 */
void mpu9250_init(Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t imuAddress, uint8_t magAddress);
#endif

/**
 * @brief Checks whether the IMU (MPU9250) was detected.
 *
 * @return true, if the IMU (MPU9250) was detected.
 */
uint8_t mpu9250_detectImu(Mpu9250* mpu);

/**
 * @brief Checks whether the magnetometer (AK8963) was detected.
 *
 * @return true if the magnetometer (AK8963) was detected.
 */
uint8_t mpu9250_detectMagnetometer(Mpu9250* mpu);

/**
 * @brief Sets the IMU to the default settings, which will suite most applications:
 * 	- The magnetometer is set to 16 bit resolution with a 100Hz sample rate
 * 	- The accelerometer is set to +-4g range with a 200Hz sample rate and a 44.8Hz DLPF
 * 	- The gyro and thermometer are set to 2000DPS range (in case of the gyro) with a 200Hz sample rate and a 41Hz DLPF
 *
 */
void mpu9250_setDefaultSettings(Mpu9250* mpu);

/**
 * @brief Calculates the gyro offsets while at rest using a lot of samples.
 */
void mpu9250_calculateGyroOffset(Mpu9250* mpu);

/**
 * @brief Enables the calculated gyro offset subtraction from the results
 *
 * @param enabled
 */
void mpu9250_enableGyroOffsetSubtraction(Mpu9250* mpu, uint8_t enabled);

/**
 * @brief Sets the sample rate divider.
 * The divider is only active when DLPF is enabled and set to 1-6 (both in case of the gyro/temp and the accelerometer).
 *
 * @param divider - the divider that divides the internal (1kHz sample rate) by (1 + divider).
 */
void mpu9250_setSampleRateDivider(Mpu9250* mpu, uint8_t divider);

/**
 * @brief Enable DLPF for the accelerometer (set fchoice_b to 0 -> fchoice to 1).
 *
 * @param enable - whether to enable the DLPF for the accelerometer.
 */
void mpu9250_enableAccDLPF(Mpu9250* mpu, uint8_t enable);

/**
 * @brief Enable DLPF for the gyro and temp sensors (set fchoice_b's to 0 -> fchoice's to 1).
 *
 * @param enable - whether to enable the DLPF for the gyro and thermometer.
 */
void mpu9250_enableGyroAndTempDLPF(Mpu9250* mpu, uint8_t enable);

/**
 * @brief Details about the values: https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
 * 	- table on page 15
 *
 * @param value - the accelerometer DLPF value
 */
void mpu9250_setAccDLPF(Mpu9250* mpu, uint8_t value);

/**
 * @brief Details about the values: https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
 * 	- table on page 13
 *
 * @param value - the gyro and thermometer DLPF value
 */
void mpu9250_setGyroAndTempDLPF(Mpu9250* mpu, uint8_t value);

/**
 * @brief Sets the gyro's sensitivity.
 *
 * @param sensitivity - possible (+/-) values are:
 * 	- 0 - 250DPS
 * 	- 1 - 500DPS
 * 	- 2 - 1000DPS
 * 	- 3 - 2000DPS
 */
void mpu9250_setGyroSensitivity(Mpu9250* mpu, uint8_t sensitivity);

/**
 * @brief Sets the accelerometer's sensitivity.
 *
 * @param sensitivity - possible (+/-) values are:
 * 	- 0 - 2G
 * 	- 1 - 4G
 * 	- 2 - 8G
 * 	- 3 - 16G
 */
void mpu9250_setAccSensitivity(Mpu9250* mpu, uint8_t sensitivity);

#ifdef ASYNC_IMU
/**
 * @brief This function is only present when using the async version of the class.
 * It needs to be called whenever the HAL_I2C_MemRxCpltCallback function is called.
 *
 */
void mpu9250_i2cReceiveCpltCallback(Mpu9250* mpu);

/**
 * @brief This function is only present when using the async version of the class.
 * Call it whenever the I2C period timer overflows.
 * It is advised to call this function at least as frequently as the highest sampling frequency set for any sensor,
 * but not too frequently - the transmission of ~24 bytes must have plenty of time to complete between two calls.
 *
 */
void mpu9250_timPeriodEllapsedCallback(Mpu9250* mpu);

/**
 * @brief This function is only present when using the async version of the class.
 * You can check whether new data is available to read using it.
 *
 * @return true, if new data is available (and clears the internal new data flag).
 */
uint8_t mpu9250_newDataAvailable(Mpu9250* mpu);
#endif

/**
 * @brief Returns a vector containing the gyro data.
 *
 * @return the gyro data in °/s.
 */
Vec3 mpu9250_readGyroData(Mpu9250* mpu);

/**
 * @brief Returns a vector containing the accelerometer data.
 *
 * @return the accelerometer data in g's.
 */
Vec3 mpu9250_readAccData(Mpu9250* mpu);

/**
 * @brief Returns the temperature data.
 *
 * @return the temperature in °C.
 */
float mpu9250_readTempData(Mpu9250* mpu);

/**
 * @brief Returns a vector containing the magnetometer data.
 *
 * @return the magnetometer data in uT.
 */
Vec3 mpu9250_readMagData(Mpu9250* mpu);


#endif /* MPU9250_MPU9250_H_ */
