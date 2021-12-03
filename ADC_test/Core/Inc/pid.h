/*
 * pid.h
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PID{
	float Kp, Ki, Kd; /* Controller gains */

	float tau; /* LP filter time constant for D-term */

	float lim_min, lim_max; /* Output limits */

	float T; /* Sample time in seconds */

	float integrator, prevErr;
	float differentiator, prevMeas;

	float out;
}PID;

void PID_Init(struct PID *pid);
float PID_Update(struct PID *pid, float setpoint, float meas);

#endif /* INC_PID_H_ */
