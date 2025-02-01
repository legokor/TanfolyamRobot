/*
 * mpu_basicOrientation.c
 *
 *  Created on: Sep 9, 2024
 *      Author: dkiovics
 */

#include "mpu_interface.h"
#include <math.h>

void mpu_updateOrientation(mpu_Mpu9250* imu, volatile mpu_Orientation* orientation, float dt) {
	mpu_Vec3 gyro = mpu_readGyroData(imu);
	mpu_Vec3 acc = mpu_readAccData(imu);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//This is the added IMU code from the videos:
	//https://youtu.be/4BoIE8YQwM8
	//https://youtu.be/j-kE0AMEWy4
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//Gyro angle calculations
	orientation->pitch += gyro.y * dt;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
	orientation->roll += gyro.x * dt;                                     //Calculate the traveled roll angle and add this to the angle_roll variable.

	//0.01745 = (3.142(PI) / 180degr)
	orientation->pitch -= orientation->roll * sin(gyro.z * 0.01745f * dt);      	//If the IMU has yawed transfer the roll angle to the pitch angel.
	orientation->roll += orientation->pitch * sin(gyro.z * 0.01745f * dt);      	//If the IMU has yawed transfer the pitch angle to the roll angel.

	//Accelerometer angle calculations
	float acc_total_vector = sqrt((acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z));    //Calculate the total accelerometer vector.

	float angle_pitch_acc = 0, angle_roll_acc = 0;

	if (fabs(acc.y) < acc_total_vector) {                                       //Prevent the asin function to produce a NaN.
		angle_pitch_acc = -asin((float)acc.x / acc_total_vector) * 57.296;      //Calculate the pitch angle.
	}
	if (fabs(acc.x) < acc_total_vector) {                                       //Prevent the asin function to produce a NaN.
		angle_roll_acc = asin((float)acc.y / acc_total_vector) * 57.296;        //Calculate the roll angle.
	}

	orientation->pitch = orientation->pitch * 0.995 + angle_pitch_acc * 0.005;    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
	orientation->roll = orientation->roll * 0.995 + angle_roll_acc * 0.005;     	//Correct the drift of the gyro roll angle with the accelerometer roll angle.
}

