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
	pid->tau = 0.001f;

	pid->timestamp_prev = get_ms();
}

/**
 * @brief Update the pid output based on error
 * @param The value to be achieved
 * @param Measured value
 */
float PID_Update(struct PID *pid, float setpoint, float meas) {

	uint32_t timestamp_now = get_ms();
	float Ts = (timestamp_now - pid->timestamp_prev) * 1e-3f;
	/* fix micros overflow */
	if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

	/* Error output from leftmost summer*/
	float err = setpoint - meas;

	/* Proportional */
	float prop = pid->Kp * err;

	/* Integrator */
	pid->integrator = pid->integrator + 0.5f * pid->Ki * Ts * (err + pid->prevErr);

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
	pid->integrator = _clamp(pid->integrator, lim_min_int, lim_max_int);

	/* Derivative: might not be necessary for FOC */
	pid->differentiator = (2.0f * pid->Kd * (meas - pid->prevMeas)
			+ (2.0f * pid->tau - pid->Ts) * pid->differentiator)
			/ (2.0f * pid->tau + pid->Ts);

	/* Output */
	pid->out = prop + pid->integrator + pid->differentiator;

	/* Limiter */
	pid->out = _clamp(pid->out, pid->lim_min, pid->lim_max);

	/* Store error and measurement in PID struct*/
	pid->prevErr = err;
	pid->prevMeas = meas;
	pid->timestamp_prev = timestamp_now;

	return pid->out;
}
