/*
 * api_robotAbstraction.h
 *
 *  Created on: Jan 31, 2025
 *      Author: dkiovics
 */

#ifndef INC_ROBOTCONTROL_API_H_
#define INC_ROBOTCONTROL_API_H_

#include <stdint.h>

#define MOT_R 0
#define MOT_L 1

typedef struct {
    uint16_t h;
    uint8_t s;
    uint8_t v;
} Color;

typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    float pitch;
    float roll;
} Orientation;

/**
 * Set motor speed
 * @param mot_lr MOT_L for left, MOT_R for right
 * @param speed between -100 and +100
 * @return 0 on success
 */
int setMotorSpeed(uint8_t mot_lr, float speed);

/**
 * Read the current value of the encoder's counter
 * @param mot_lr MOT_L for left, MOT_R for right motor
 * @return encoder counter value
 */
int getEncoderPosition(uint8_t mot_lr);

/**
 * Prints text to the LCD. Works just like printf after the position specifiers
 * WARNING: because of limited available flash memory, currently only either *printf
 * 			or *scanf can support floats
 * 			this can be changed under Project -> Properties -> C/C++ build -> Settings -> MCU/MPU settings
 * DEFAULT: only *scanf float support
 *
 * @param row of starting position
 * @param col of starting position
 * @param fmt printf-like format string followed by a variable number of arguments
 */
int lcdPrintf(uint8_t row, uint8_t col, const char *fmt, ...);

/**
 * Print to the USB serial port. Works just like regular printf.
 * WARNING: because of limited available flash memory, currently only either *printf
 * 			or *scanf can support floats
 * 			this can be changed under Project -> Properties -> C/C++ build -> Settings -> MCU/MPU settings
 * DEFAULT: only *scanf float support
 *
 * @param fmt printf-like format string followed by a variable number of arguments
 */
int uartPrintf(const char *fmt, ...);

/**
 * Print to the telemetry webpage. Works just like regular printf.
 * WARNING: because of limited available flash memory, currently only either *printf
 * 			or *scanf can support floats
 * 			this can be changed under Project -> Properties -> C/C++ build -> Settings -> MCU/MPU settings
 * DEFAULT: only *scanf float support
 *
 * @param fmt printf-like format string followed by a variable number of arguments
 */
int espPrintf(const char *fmt, ...);

/**
 * Read input from the telemetry webpage console input.
 * @param data pointer to the character buffer where we want to get the output
 * @return 1 if there is data available, else returns 0
 */
int espRead(char* data);

/**
 * Read color in HSV format
 * @param color the Color struct pointer in which we want to get the result
 */
void getColorHsv(Color* color);

/**
 * Set the servo's position in degrees
 * @param position must be between -90 and 90. Otherwise it will be clipped to those values.
 */
void setServoPosition(int8_t position);

#if US_SENSOR
/**
 * Get the distance measured by the ultrasonic ranging sensor
 * @return distance in cm
 */
uint16_t getUsDistance();
#elif IR_SENSOR
/**
 * Get the distance measured by the infrared ranging sensor
 * @return distance in mm
 */
uint16_t getIrDistance();
#else
	#error "No ranging module defined as active"
#endif

/**
 * Get the accelerometer data for all 3 axis. All values are in the +-4g range
 * @param acc a pointer to a Vec3 struct where the data will be stored
 */
void getAccData(Vec3* acc);

/**
 * Get the gyroscope data for all 3 axis. All values are in the +-4000°/s range
 * @param gyro a pointer to a Vec3 struct where the data will be stored
 */
void getGyroData(Vec3* gyro);

/**
 * Get the magnetometer data for all 3 axis.
 * @param mag a pointer to a Vec3 struct where the data will be stored
 */
void getMagData(Vec3* mag);

/**
 * Get the robot's orientation in terms of pitch and roll (in degrees)
 * @param orientation a pointer to an Orientation struct where the data will be stored
 */
void getOrientation(Orientation* orientation);

/**
 * Get the temperature of the IMU IC.
 */
float getTemp();

/**
 * Do nothing for the specified time
 * @param delay in ms
 */
void delayMs(uint32_t delay);

/**
 * Do nothing for the specified time
 * @param delay in us
 * WARNING: the delay time must not exceed 18000us!
 */
void delayUs(uint32_t delay);

/**
 * Returns the number of milliseconds that have passed since the last reset
 * WARNING: the precision is only +-1Ms
 */
uint32_t getTimeMs();

#endif /* INC_ROBOTCONTROL_API_H_ */
