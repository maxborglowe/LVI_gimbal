/*
 * Quaternions.c
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#include <math.h>
#include "Quaternions.h"

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z){
	// Local system variables
	float norm;
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
	float f_1, f_2, f_3;
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;

	// Axulirary variables to avoid repeated calculations
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3;
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	//normalize the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;

	// Compute the quaternion derivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// Compute then integrate the estimated quaternion derivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
	// Normalize quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);

	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}

struct EulerAngles ToEulerAngles(float q1, float q2, float q3, float q4) {
    struct EulerAngles angles;

    // roll (x-axis rotation)
//    angles.x = atan2(2.0);

    double sinr_cosp = 2 * (q1 * q2 + q3 * q4);
    double cosr_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q1 * q3 - q4 * q2);
    if (abs((int)sinp) >= 1)
        angles.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q1 * q4 + q2 * q3);
    double cosy_cosp = 1 - 2 * (q3 * q3 + q4 * q4);
    angles.z = atan2(siny_cosp, cosy_cosp);

    return angles;
}

/*
 * Fast inverse Square root
 * @param the input value which will be converted to its inverse sqrt
 */
float invSqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}
