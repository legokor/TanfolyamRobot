/*
 * mpu_mpu9250.c
 *
 *  Created on: May 20, 2024
 *      Author: dkiovics
 */

#include "mpu_interface.h"
#include <string.h>


#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00		// should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09
#define AK8963_CNTL      0x0A
#define AK8963_ASTC      0x0C
#define AK8963_I2CDIS    0x0F
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20
#define ZMOT_THR         0x21
#define ZRMOT_DUR        0x22

#define FIFO_EN          0x23
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define MOT_DETECT_STATUS 0x61
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D
#define DMP_RW_PNT       0x6E
#define DMP_REG          0x6F
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 		// Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


#ifdef ASYNC_IMU
/**
 * @brief This function is only present when using the async version of the class.
 * It needs to be called whenever the HAL_I2C_MemRxCpltCallback function is called.
 *
 */
void mpu_i2cReceiveCpltCallback(mpu_Mpu9250* mpu){
	if(mpu->isReadingImu){
		memcpy((void*)mpu->imuData, (void*)mpu->imuBuffer, 14);
		mpu->isReadingImu = 0;
		if(mpu->readEnabled)
			HAL_I2C_Mem_Read_IT(mpu->hi2c, mpu->magAddress, AK8963_XOUT_L, 1, (uint8_t*)mpu->magBuffer, 7);
		else
			mpu->readInProgress = 0;
	}else{
		memcpy((void*)mpu->magData, (void*)mpu->magBuffer, 6);
		mpu->readInProgress = 0;
		mpu->newData = 1;
	}
}

/**
 * @brief This function is only present when using the async version of the class.
 * Call it whenever the I2C period timer overflows.
 * It is advised to call this function at least as frequently as the highest sampling frequency set for any sensor,
 * but not too frequently - the transmission of ~24 bytes must have plenty of time to complete between two calls.
 *
 */
void mpu_timPeriodEllapsedCallback(mpu_Mpu9250* mpu){
	if(!mpu->readEnabled || !mpu->initComplete)
		return;
	mpu->readInProgress = 1;
	mpu->isReadingImu = 1;
	HAL_I2C_Mem_Read_IT(mpu->hi2c, mpu->imuAddress, ACCEL_XOUT_H, 1, (uint8_t*)mpu->imuBuffer, 14);
}

/**
 * @brief This function is only present when using the async version of the class.
 * You can check whether new data is available to read using it.
 *
 * @return true, if new data is available (and clears the internal new data flag).
 */
uint8_t mpu_newDataAvailable(mpu_Mpu9250* mpu){
	uint8_t tmp = mpu->newData;
	if(tmp)
		mpu->newData = 0;
	return mpu->newData;
}
#endif


static uint8_t mpu_writeBlocking(mpu_Mpu9250* mpu, uint8_t devAddress, uint8_t regAddress, uint8_t data){
#ifdef ASYNC_IMU
	mpu->readEnabled = 0;
	while(mpu->readInProgress) {}
#endif
	uint8_t txData[] = {regAddress, data};
	uint8_t ok = 0;
	while(1) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(mpu->hi2c, devAddress, txData, 2, 1000);
		if(status != HAL_BUSY){
			ok = status == HAL_OK;
			break;
		}
	}
#ifdef ASYNC_IMU
	mpu->readEnabled = 1;
#endif
	return ok;
}

static uint8_t mpu_readBlocking(mpu_Mpu9250* mpu, uint8_t devAddress, uint8_t regAddress, uint8_t numBytes, volatile uint8_t* buffer){
#ifdef ASYNC_IMU
	mpu->readEnabled = 0;
	while(mpu->readInProgress) {}
#endif
	uint8_t ok = 0;
	while(1) {
		HAL_StatusTypeDef status = HAL_I2C_Mem_Read(mpu->hi2c, devAddress, regAddress, 1, (uint8_t*)buffer, numBytes, 1000);
		if(status != HAL_BUSY){
			ok = status == HAL_OK;
			break;
		}
	}
#ifdef ASYNC_IMU
	mpu->readEnabled = 1;
#endif
	return ok;
}


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
void mpu_init(mpu_Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t p_imuAddress, uint8_t p_magAddress, IRQn_Type readIr) {
#else
/**
 * @brief Constructs the Mpu9250 class for sync (blocking) data retrieval.
 *
 * @param hi2c - the I2C HAL object pointer.
 * @param imuAddress - the IMU IC I2C address (in reality this is the MPU9250 IC, but the IC's package also contains
 * an AK8963 magnetometer IC).
 * @param magAddress - the magnetometer IC I2C address (generally it is 0x0C).
 */
void mpu_init(mpu_Mpu9250* mpu, I2C_HandleTypeDef* hi2c, uint8_t p_imuAddress, uint8_t p_magAddress) {
	mpu->readIr = readIr;
	mpu->isReadingImu = 0;
	mpu->newData = 0;
	mpu->readEnabled = 0;
	mpu->readInProgress = 0;
#endif
	mpu->imuAddress = p_imuAddress << 1;
	mpu->magAddress = p_magAddress << 1;
	mpu->hi2c = hi2c;
	mpu->initComplete = 0;

	mpu->gyroOffsetX = mpu->gyroOffsetY = mpu->gyroOffsetZ = 0;
	mpu->useGyroOffsets = 0;

	mpu_writeBlocking(mpu, mpu->imuAddress, PWR_MGMT_1, 0x00);		//RESET, enable all sensors
	HAL_Delay(100);
	mpu_writeBlocking(mpu, mpu->imuAddress, PWR_MGMT_1, 0x01);		//Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	HAL_Delay(100);
	mpu_writeBlocking(mpu, mpu->imuAddress, INT_PIN_CFG, 0x22);		//Enable bypass to magnetometer
	HAL_Delay(100);

	mpu_writeBlocking(mpu, mpu->magAddress, AK8963_CNTL, 0x00);		//Power down magnetometer
	HAL_Delay(10);
	mpu_writeBlocking(mpu, mpu->magAddress, AK8963_CNTL, 0x0F);		//Enter fuse access mode
	HAL_Delay(10);
	uint8_t temp[3];
	mpu_readBlocking(mpu, mpu->magAddress, AK8963_ASAX, 3, temp);	//Read calibration values
	mpu->magCoeff_x = (((float)temp[0] - 128.0) / 256.0) + 1;
	mpu->magCoeff_y = (((float)temp[1] - 128.0) / 256.0) + 1;
	mpu->magCoeff_z = (((float)temp[2] - 128.0) / 256.0) + 1;
	mpu_writeBlocking(mpu, mpu->magAddress, AK8963_CNTL, 0);		//Power down magnetometer
	HAL_Delay(10);
	mpu_writeBlocking(mpu, mpu->magAddress, AK8963_CNTL, 0x16);		//16 bit resolution, continuous measurement at 100Hz
	HAL_Delay(10);

	uint8_t imuDetected = mpu_detectImu(mpu);
	uint8_t magDetected = mpu_detectMagnetometer(mpu);

	mpu->initComplete = imuDetected && magDetected;
#ifdef ASYNC_IMU
	mpu->readEnabled = mpu->initComplete;
#endif
}

/**
 * @brief Calculates the gyro offsets while at rest using a lot of samples.
 */
void mpu_calculateGyroOffset(mpu_Mpu9250* mpu){
	if(!mpu->initComplete){
		return;
	}
	uint8_t prevEnabled = mpu->useGyroOffsets;
	mpu->useGyroOffsets = 0;
	mpu->gyroOffsetX = mpu->gyroOffsetY = mpu->gyroOffsetZ = 0;
	for(int p = 0; p < 200; p++){
		mpu_Vec3 res;
		res = mpu_readGyroData(mpu);
		mpu->gyroOffsetX += res.x;
		mpu->gyroOffsetY += res.y;
		mpu->gyroOffsetZ += res.z;
		HAL_Delay(8);
	}
	mpu->gyroOffsetX /= 200;
	mpu->gyroOffsetY /= 200;
	mpu->gyroOffsetZ /= 200;
	mpu->useGyroOffsets = prevEnabled;
}

/**
 * @brief Enables the calculated gyro offset subtraction from the results
 *
 * @param enabled
 */
void mpu_enableGyroOffsetSubtraction(mpu_Mpu9250* mpu, uint8_t enabled){
	if(!mpu->initComplete){
		return;
	}
	mpu->useGyroOffsets = enabled;
}

/**
 * @brief Checks whether the IMU (MPU9250) was detected.
 *
 * @return true, if the IMU (MPU9250) was detected.
 */
uint8_t mpu_detectImu(mpu_Mpu9250* mpu){
	uint8_t whoAmI;
	mpu_readBlocking(mpu, mpu->imuAddress, WHO_AM_I_MPU9250, 1, &whoAmI);
	return whoAmI == 0x71;
}

/**
 * @brief Checks whether the magnetometer (AK8963) was detected.
 *
 * @return true if the magnetometer (AK8963) was detected.
 */
uint8_t mpu_detectMagnetometer(mpu_Mpu9250* mpu){
	uint8_t whoAmI;
	mpu_readBlocking(mpu, mpu->magAddress, AK8963_WHO_AM_I, 1, &whoAmI);
	return whoAmI == 0x48;
}

/**
 * @brief Sets the IMU to the default settings, which will suite most applications:
 * 	- The magnetometer is set to 16 bit resolution with a 100Hz sample rate
 * 	- The accelerometer is set to +-4g range with a 200Hz sample rate and a 44.8Hz DLPF
 * 	- The gyro and thermometer are set to 2000DPS range (in case of the gyro) with a 200Hz sample rate and a 41Hz DLPF
 *
 */
void mpu_setDefaultSettings(mpu_Mpu9250* mpu){
	if(!mpu->initComplete)
		return;

	mpu_setGyroSensitivity(mpu, 3);			//Set gyro full scale range (+-2000DPS)
	mpu_enableGyroAndTempDLPF(mpu, 1);		//Enable DLPF for the gyro and temp sensors (set fchoice_b's to 0 -> fchoice's to 1)
	mpu_setGyroAndTempDLPF(mpu, 3);			//Set gyro and temp DLPF to 41Hz (results in a 5.9ms delay and a 1kHz sample rate)

	mpu_setAccSensitivity(mpu, 1);			//Set accelerometer sensitivity to +-4g
	mpu_enableAccDLPF(mpu, 1);				//Enable DLPF for accelerometer (set fchoice_b to 0 -> fchoice to 1)
	mpu_setAccDLPF(mpu, 3);					//Set accelerometer DLPF to 44.8Hz (results in a 4.88ms delay and a 1kHz sample rate)

	mpu_setSampleRateDivider(mpu, 4);		//Set the sample rate divider to 4+1=5 (so that the gyro/temp and accelerometer data rate is 200Hz)
}

/**
 * @brief Sets the sample rate divider.
 * The divider is only active when DLPF is enabled and set to 1-6 (both in case of the gyro/temp and the accelerometer).
 *
 * @param divider - the divider that divides the internal (1kHz sample rate) by (1 + divider).
 */
void mpu_setSampleRateDivider(mpu_Mpu9250* mpu, uint8_t divider){
	if(!mpu->initComplete)
		return;

	mpu_writeBlocking(mpu, mpu->imuAddress, SMPLRT_DIV, divider);
}

/**
 * @brief Enable DLPF for the accelerometer (set fchoice_b to 0 -> fchoice to 1).
 *
 * @param enable - whether to enable the DLPF for the accelerometer.
 */
void mpu_enableAccDLPF(mpu_Mpu9250* mpu, uint8_t enable){
	if(!mpu->initComplete)
		return;

	uint8_t accConfigTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG2, 1, &accConfigTmp);
	accConfigTmp &= 0xf7;
	if(enable)
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG2, accConfigTmp | 0x00);
	else
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG2, accConfigTmp | 0x08);
}

/**
 * @brief Enable DLPF for the gyro and temp sensors (set fchoice_b's to 0 -> fchoice's to 1).
 *
 * @param enable - whether to enable the DLPF for the gyro and thermometer.
 */
void mpu_enableGyroAndTempDLPF(mpu_Mpu9250* mpu, uint8_t enable){
	if(!mpu->initComplete)
		return;

	uint8_t gyroConfigTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, 1, &gyroConfigTmp);
	gyroConfigTmp &= 0xfc;
	if(enable)
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, gyroConfigTmp | 0x00);
	else
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, gyroConfigTmp | 0x03);
}

/**
 * @brief Details about the values: https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
 * 	- table on page 15
 *
 * @param value - the accelerometer DLPF value
 */
void mpu_setAccDLPF(mpu_Mpu9250* mpu, uint8_t value){
	if(!mpu->initComplete)
		return;

	uint8_t accConfigTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG2, 1, &accConfigTmp);
	accConfigTmp &= 0xf8;
	mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG2, accConfigTmp | value);
}

/**
 * @brief Details about the values: https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
 * 	- table on page 13
 *
 * @param value - the gyro and thermometer DLPF value
 */
void mpu_setGyroAndTempDLPF(mpu_Mpu9250* mpu, uint8_t value){
	if(!mpu->initComplete)
		return;

	uint8_t configTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, CONFIG, 1, &configTmp);
	configTmp &= 0xf8;
	mpu_writeBlocking(mpu, mpu->imuAddress, CONFIG, configTmp | value);
}

/**
 * @brief Sets the accelerometer's sensitivity.
 *
 * @param sensitivity - possible (+/-) values are:
 * 	- 0 - 2G
 * 	- 1 - 4G
 * 	- 2 - 8G
 * 	- 3 - 16G
 */
void mpu_setAccSensitivity(mpu_Mpu9250* mpu, uint8_t sensitivity){
	if(!mpu->initComplete)
		return;

	uint8_t accConfigTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG, 1, &accConfigTmp);
	accConfigTmp &= 0xe7;
	switch(sensitivity) {
	case 0:
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG, (0x00 << 3) | accConfigTmp);
		mpu->accSensitivity = 2.0 / 32768.0;
		break;
	case 1:
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG, (0x01 << 3) | accConfigTmp);
		mpu->accSensitivity = 4.0 / 32768.0;
		break;
	case 2:
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG, (0x02 << 3) | accConfigTmp);
		mpu->accSensitivity = 8.0 / 32768.0;
		break;
	case 3:
		mpu_writeBlocking(mpu, mpu->imuAddress, ACCEL_CONFIG, (0x03 << 3) | accConfigTmp);
		mpu->accSensitivity = 16.0 / 32768.0;
		break;
	}
}

/**
 * @brief Sets the gyro's sensitivity.
 *
 * @param sensitivity - possible (+/-) values are:
 * 	- 0 - 250DPS
 * 	- 1 - 500DPS
 * 	- 2 - 1000DPS
 * 	- 3 - 2000DPS
 */
void mpu_setGyroSensitivity(mpu_Mpu9250* mpu, uint8_t sensitivity){
	if(!mpu->initComplete)
		return;

	uint8_t gyroConfigTmp;
	mpu_readBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, 1, &gyroConfigTmp);
	gyroConfigTmp &= 0xe7;
	switch(sensitivity) {
	case 0:
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, (0x00 << 3) | gyroConfigTmp);
		mpu->gyroSensitivity = 250.0 / 32768.0;
		break;
	case 1:
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, (0x01 << 3) | gyroConfigTmp);
		mpu->gyroSensitivity = 500.0 / 32768.0;
		break;
	case 2:
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, (0x02 << 3) | gyroConfigTmp);
		mpu->gyroSensitivity = 1000.0 / 32768.0;
		break;
	case 3:
		mpu_writeBlocking(mpu, mpu->imuAddress, GYRO_CONFIG, (0x03 << 3) | gyroConfigTmp);
		mpu->gyroSensitivity = 2000.0 / 32768.0;
		break;
	}
}

/**
 * @brief Returns a vector containing the gyro data.
 *
 * @return the gyro data in °/s.
 */
mpu_Vec3 mpu_readGyroData(mpu_Mpu9250* mpu){
	if(!mpu->initComplete){
		mpu_Vec3 null;
		null.x = 0;
		null.y = 0;
		null.z = 0;
		return null;
	}

#ifndef ASYNC_IMU
	mpu_readBlocking(mpu->imuAddress, GYRO_XOUT_H, 6, mpu->imuData + 8);
#else
	HAL_NVIC_DisableIRQ(mpu->readIr);
#endif
	int16_t x = mpu->imuData[8] << 8 | mpu->imuData[9];
	int16_t y = mpu->imuData[10] << 8 | mpu->imuData[11];
	int16_t z = mpu->imuData[12] << 8 | mpu->imuData[13];
#ifdef ASYNC_IMU
	HAL_NVIC_EnableIRQ(mpu->readIr);
#endif
	mpu_Vec3 data;
	data.x = x * mpu->gyroSensitivity;
	data.y = y * mpu->gyroSensitivity;
	data.z = z * mpu->gyroSensitivity;
	if(mpu->useGyroOffsets){
		data.x -= mpu->gyroOffsetX;
		data.y -= mpu->gyroOffsetY;
		data.z -= mpu->gyroOffsetZ;
	}
	return data;
}

/**
 * @brief Returns a vector containing the accelerometer data.
 *
 * @return the accelerometer data in g's.
 */
mpu_Vec3 mpu_readAccData(mpu_Mpu9250* mpu){
	if(!mpu->initComplete){
		mpu_Vec3 null;
		null.x = 0;
		null.y = 0;
		null.z = 0;
		return null;
	}

#ifndef ASYNC_IMU
	mpu_readBlocking(mpu->imuAddress, ACCEL_XOUT_H, 6, mpu->imuData);
#else
	HAL_NVIC_DisableIRQ(mpu->readIr);
#endif
	int16_t x = mpu->imuData[0] << 8 | mpu->imuData[1];
	int16_t y = mpu->imuData[2] << 8 | mpu->imuData[3];
	int16_t z = mpu->imuData[4] << 8 | mpu->imuData[5];
#ifdef ASYNC_IMU
	HAL_NVIC_EnableIRQ(mpu->readIr);
#endif
	mpu_Vec3 data;
	data.x = x * mpu->accSensitivity;
	data.y = y * mpu->accSensitivity;
	data.z = z * mpu->accSensitivity;
	return data;
}

/**
 * @brief Returns the temperature data.
 *
 * @return the temperature in °C.
 */
float mpu_readTempData(mpu_Mpu9250* mpu){
	if(!mpu->initComplete)
		return 0;

#ifndef ASYNC_IMU
	mpu_readBlocking(mpu->imuAddress, TEMP_OUT_H, 2, mpu->imuData + 6);
#else
	HAL_NVIC_DisableIRQ(mpu->readIr);
#endif
	int16_t t = mpu->imuData[6] << 8 | mpu->imuData[7];
#ifdef ASYNC_IMU
	HAL_NVIC_EnableIRQ(mpu->readIr);
#endif
	return (t - 21.0) / 333.87 + 21.0;
}

/**
 * @brief Returns a vector containing the magnetometer data.
 *
 * @return the magnetometer data in uT.
 */
mpu_Vec3 mpu_readMagData(mpu_Mpu9250* mpu){
	if(!mpu->initComplete){
		mpu_Vec3 null;
		null.x = 0;
		null.y = 0;
		null.z = 0;
		return null;
	}

#ifndef ASYNC_IMU
	mpu_readBlocking(mpu->magAddress, AK8963_XOUT_L, 7, mpu->magData);
#else
	HAL_NVIC_DisableIRQ(mpu->readIr);
#endif
	int16_t x = mpu->magData[1] << 8 | mpu->magData[0];
	int16_t y = mpu->magData[3] << 8 | mpu->magData[2];
	int16_t z = mpu->magData[5] << 8 | mpu->magData[4];
#ifdef ASYNC_IMU
	HAL_NVIC_EnableIRQ(mpu->readIr);
#endif
	const float sensitivity = 4912.0 / 32768.0;
	mpu_Vec3 data;
	data.x = x * mpu->magCoeff_x * sensitivity;
	data.y = y * mpu->magCoeff_y * sensitivity;
	data.z = z * mpu->magCoeff_z * sensitivity;
	return data;
}


