/*
 * Quaternions.c
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#include <math.h>
#include "Quaternions.h"
#include "time_utils.h"

/*
 * @brief Set the sample frequency based on the while loop cycle time (in milliseconds)
 */
<<<<<<< HEAD:Core/Src/Quaternions.c
void setSampleFreq_ms() {
	sampleFreq = 1 / (1e-3 * while_t);
}

/*
 * @brief Set the sample frequency based on the while loop cycle time (in milliseconds)
 */
void setSampleFreq_us() {

	sampleFreq = 1 / get_us();
//	sampleFreq = 1 / (1e-6 * us_t);
=======
void setSampleFreq() {
	sampleFreq = 1 / (0.001 * while_t);
>>>>>>> 26af4804f429aa13389f90c04332cd7e0c0d610c:BLDC_Gimbal1/Core/Src/Quaternions.c
}

/*
 * @brief Convert gyroscope and accelerometer data to quaternions using Madgwick's algorithm.
 * @param Current angular rate on x-axis from gyroscope
 * @param Current angular rate on y-axis from gyroscope
 * @param Current angular rate on z-axis from gyroscope
 * @param Current acceleration on x-axis from accelerometer
 * @param Current acceleration on y-axis from accelerometer
 * @param Current acceleration on z-axis from accelerometer
 */
void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az) {

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1,
			q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
				+ _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
				+ _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / (uint16_t) sampleFreq);
	q1 += qDot2 * (1.0f / (uint16_t) sampleFreq);
	q2 += qDot3 * (1.0f / (uint16_t) sampleFreq);
	q3 += qDot4 * (1.0f / (uint16_t) sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/*
 * @brief Convert quaternions to Euler angles
 * @param Real component
 * @param i-component
 * @param j-component
 * @param k-component
 */
struct EulerAngles ToEulerAngles(float _q0, float _q1, float _q2, float _q3) {
	struct EulerAngles angles;

	// roll (x-axis rotation)
	angles.x = atan2(2 * (_q0 * _q1 + _q2 * _q3),
<<<<<<< HEAD:Core/Src/Quaternions.c
			_q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3);
=======
			1 - 2 * (_q1 * _q1 + _q2 * _q2));
>>>>>>> 26af4804f429aa13389f90c04332cd7e0c0d610c:BLDC_Gimbal1/Core/Src/Quaternions.c

	// pitch (y-axis rotation)
	angles.y = -asin(2 * (_q1 * _q3 - _q0 * _q2));
//	double sinp = 2 * (_q0 * _q2 - _q3 * _q1);
//	if (abs((int) sinp) >= 1)
//		angles.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//	else
//		angles.y = asin(sinp);

	// yaw (z-axis rotation)
	angles.z = atan2(2 * (_q0 * _q3 + _q1 * _q2),
<<<<<<< HEAD:Core/Src/Quaternions.c
			_q0 * _q0 + _q1 * _q1 - _q2 * _q2 - _q3 * _q3);
=======
			1 - 2 * (_q2 * _q2 + _q3 * _q3));
>>>>>>> 26af4804f429aa13389f90c04332cd7e0c0d610c:BLDC_Gimbal1/Core/Src/Quaternions.c

	return angles;
}

/*
 * @brief Fast inverse Square root
 * @param The input value which will be converted to its inverse sqrt
 */
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}
