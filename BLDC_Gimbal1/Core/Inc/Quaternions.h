/*
 * Quaternions.h
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#ifndef INC_QUATERNIONS_H_
#define INC_QUATERNIONS_H_


//extern const float dt;
//extern float g_dps;
//extern float  g_err;
//extern float gyroMeasError;
//extern float beta;

extern float SEq_1, SEq_2, SEq_3, SEq_4;
extern float sampleDelay;
extern

#define dt 1/sampleDelay //has to be changed according to the sample rate of the IMU
#define g_dps 2000.0f
#define g_err 0.01f * g_dps //error = 1% of e
#define gyroMeasError M_PI * (g_err / 180.0f)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError

struct EulerAngles{
	float x;
	float y;
	float z;
};

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
struct EulerAngles ToEulerAngles(float q1, float q2, float q3, float q4);
float invSqrt(float x);

#endif /* INC_QUATERNIONS_H_ */
