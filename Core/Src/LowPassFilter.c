/*
 * LPF.c
 *
 *  Created on: Jan 28, 2022
 *      Author: maxborglowe
 */

#include "LowPassFilter.h"

void lpf_init(struct LPF *lpf, float Tf_init){
	lpf->Tf = Tf_init;
	lpf->out_prev = 0;
	lpf->timestamp_prev = get_ms();
}

float lpf_exec(struct LPF *lpf, float input){
	uint32_t timestamp = get_ms();
	float dt = (timestamp - lpf->timestamp_prev) * 1e-3f;

	if (dt < 0.0f ) dt = 1e-3f;
	else if(dt > 0.3f) {
		lpf->out_prev = input;
		lpf->timestamp_prev = timestamp;
		return input;
	}

	float alpha = lpf->Tf/(lpf->Tf + dt);
	float out = alpha * lpf->out_prev + (1.0f - alpha) * input;
	lpf->out_prev = out;
	lpf->timestamp_prev = timestamp;
	return out;
}



