/*
 * pid.h
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "def.h"
#include "time_utils.h"

typedef struct PID {
	float Kp, Ki, Kd; /* Controller gains */

	float lim_min, lim_max; /* Output limits */

	uint32_t timestamp_prev;
	float Ts; /* Sample time in seconds */
	float tau; /* Derivative filter time constant */

	float integrator, prevErr;
	float differentiator, prevMeas;

	float out;
} PID;

void PID_Init(struct PID *pid);
float PID_Update(struct PID *pid, float setpoint, float meas);

#endif /* INC_PID_H_ */
