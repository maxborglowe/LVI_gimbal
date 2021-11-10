/*
 * Quaternions.h
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#ifndef INC_QUATERNIONS_H_
#define INC_QUATERNIONS_H_

extern float SEq_1, SEq_2, SEq_3, SEq_4;
extern volatile float q0, q1, q2, q3;
extern float sampleDelay;
extern float while_t;

float sampleFreq;								//the rate at which the IMU is being sampled in while loop
												//+ extra time for other stuff
#define beta 0.1f								//proportional gain

struct EulerAngles {
	float x;
	float y;
	float z;
};

void setSampleFreq(void);
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y,
		float a_z);
struct EulerAngles ToEulerAngles(float q1, float q2, float q3, float q4);
float invSqrt(float x);

#endif /* INC_QUATERNIONS_H_ */
