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
	pid->Kp = 0.0f;
	pid->Ki = 0.0f;
	pid->Kd = 0.0f;

	pid->timestamp_prev = get_us();
}

/**
 * @brief Update the pid output based on error
 * @param The value to be achieved
 * @param Measured value
 */
float PID_Update(struct PID *pid, float setpoint, float meas) {

	uint32_t timestamp_now = get_us();
	float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
	/* fix micros overflow */
	if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

	/* Error output from leftmost summer*/
	float err = setpoint - meas;

	/* Proportional */
	float prop = pid->Kp * err;

	/* Integrator */
	pid->integrator = pid->integrator + 0.5f * pid->Ki * Ts * (err + pid->prevErr);

	/* Anti-windup scheme */
//	float lim_min_int, lim_max_int;
//	if (pid->lim_max > prop) {
//		lim_max_int = pid->lim_max - prop;
//	} else {
//		lim_max_int = 0.0f;
//	}
//
//	if (pid->lim_min < prop) {
//		lim_min_int = pid->lim_min - prop;
//	} else {
//		lim_min_int = 0.0f;
//	}

	pid->integrator = _constrain(pid->integrator, pid->lim_min, pid->lim_max);

	/* Clamp integrator -> prevents integrator from growing out of proportions */
//	if (pid->integrator > lim_max_int) {
//		pid->integrator = lim_max_int;
//	} else if (pid->integrator < lim_min_int) {
//		pid->integrator = lim_min_int;
//	}

	/* Derivative: not necessary for FOC */

	/* Output */
	pid->out = prop + pid->integrator;

	/* Limiter */
//	if (pid->out > pid->lim_max) {
//		pid->out = pid->lim_max;
//	} else if (pid->out < pid->lim_min) {
//		pid->out = pid->lim_min;
//	}

	pid->out = _constrain(pid->out, pid->lim_min, pid->lim_max);

	/* Store error and measurement in PID struct*/
	pid->prevErr = err;
	pid->prevMeas = meas;
	pid->timestamp_prev = timestamp_now;

	return pid->out;
}
