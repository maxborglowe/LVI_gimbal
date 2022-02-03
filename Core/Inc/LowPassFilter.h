/*
 * LPF.h
 *
 *  Created on: Jan 28, 2022
 *      Author: maxborglowe
 */

#ifndef INC_LOWPASSFILTER_H_
#define INC_LOWPASSFILTER_H_

#include "time_utils.h"

typedef struct LPF {
	uint32_t timestamp_prev; 	/* Timestamp from previous execution */
	float out_prev;				/* Output value from previous execution */
	float Tf;					/* Filter time constant */
} LPF;

void lpf_init(struct LPF *lpf, float Tf_init);
float lpf_exec(struct LPF *lpf, float input);

#endif /* INC_LOWPASSFILTER_H_ */
