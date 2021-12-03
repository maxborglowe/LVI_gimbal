/*
 * pid.c
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#include "pid.h"


void PID_Init(struct PID *pid) {
	pid->integrator = 0.0f;
	pid->prevErr = 0.0f;
	pid->differentiator = 0.0f;
	pid->prevMeas = 0.0f;
	pid->out = 0.0f;
}

float PID_Update(struct PID *pid, float setpoint, float meas) {
	/* Error output from leftmost summer*/
	float err = setpoint - meas;

	/* Proportional */
	float prop = pid->Kp * err;

	/* Integrator */
	pid->integrator = pid->integrator
			+ 0.5f * pid->Ki * pid->T * (err + pid->prevErr);

	/* Anti-windup scheme */
	float lim_min_int, lim_max_int;
	if (pid->lim_max > prop) {
		lim_max_int = pid->lim_max - prop;
	} else {
		lim_max_int = 0.0f;
	}

	if (pid->lim_min < prop) {
		lim_min_int = pid->lim_min - prop;
	} else {
		lim_min_int = 0.0f;
	}

	/* Clamp integrator -> prevents integrator from growing out of proportions */
	if (pid->integrator > lim_max_int) {
		pid->integrator = lim_max_int;
	} else if (pid->integrator < lim_min_int) {
		pid->integrator = lim_min_int;
	}

	/* Derivative */

	/* Output */
	pid->out = prop + pid->integrator + pid->differentiator;

	/* Limiter */
	if(pid->out > pid->lim_max){
		pid->out = pid->lim_max;
	} else if(pid->out < pid->lim_min){
		pid->out = pid->lim_min;
	}

	/* Store error and measurement in PID struct*/
	pid->prevErr = err;
	pid->prevMeas = meas;

	return pid->out;
}
