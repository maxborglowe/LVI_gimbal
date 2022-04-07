/*
 * sincos.h
 *
 *  Created on: Mar 28, 2022
 *      Author: maxborglowe
 */

#ifndef INC_FASTTRIGONOMETRY_H_
#define INC_FASTTRIGONOMETRY_H_

#include <stdint.h>

#define MAX_CIRCLE_ANGLE 8192 /** Sin/cos approximation resolution Note: larger values --> more memory required */
#define HALF_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE - 1)

void FastTrigonometry_buildTable();
float FastTrigonometry_cos(float x);
float FastTrigonometry_sin(float x);
float FastTrigonometry_atan2(float y, float x);

#endif /* INC_FASTTRIGONOMETRY_H_ */
