/*
 * Quaternions.h
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#ifndef INC_QUATERNIONS_H_
#define INC_QUATERNIONS_H_

#include <stdio.h>
#include "time_utils.h"

extern float SEq_1, SEq_2, SEq_3, SEq_4;
extern volatile float q0, q1, q2, q3;

float sampleFreq;	//the rate at which the IMU is being sampled in while loop
extern float sampleFreq_inv;
//+ extra time for other stuff
#define beta 0.1f								//proportional gain

typedef struct EulerAngles {
	float x; /* Roll */
	float y; /* Pitch */
	float z; /* Yaw */
} EulerAngles;

typedef struct Quaternion {
	float q0, q1, q2, q3;
} Quaternion;

float setSampleFreq_ms(void);
float setSampleFreq_us(uint32_t us);
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float time);
struct EulerAngles ToEulerAngles(float q1, float q2, float q3, float q4);
float invSqrt(float x);

#endif /* INC_QUATERNIONS_H_ */
