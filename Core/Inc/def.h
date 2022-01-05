/*
 * global_def.h
 *
 *  Created on: Nov 22, 2021
 *      Author: maxborglowe
 *
 *      Global definitions for gimbal program
 */
#include <stdio.h>
#include <math.h>
#include "Quaternions.h"

#ifndef INC_DEF_H_
#define INC_DEF_H_

/*Configuration vars
 * Set these to configure what is read, written, printed, etc. in the while loop
 * 0 = disable function, 1 = enable function*/

//######################################
#define USE_SIM 0
#define USE_ICM20602 0
#define USE_BMI270 0
#define USE_AS5048A 1
#define USE_DRV8313 1
#define USE_PRINT 0
#define USE_IMU_VIS 0
//######################################

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define AMT_MOTORS 3
#define CH_PER_MOTOR 3
#define ADC_CHANNELS AMT_MOTORS * CH_PER_MOTOR

#define _1_SQRT2 0.7071067
#define _1_SQRT3 0.5773503
#define _SQRT3 1.7320501
#define _rl_e_j2PI_3 -0.5f
#define _im_e_j2PI_3 0.866025404f
#define _rl_e_j4PI_3 -0.5f
#define _im_e_j4PI_3 -0.866025404f
#define _1_3 0.33333333f
#define _2_3 0.66666667f
#define _1PI_3 M_PI*_1_3
#define _2PI_3 M_PI*_2_3
#define _PI M_PI
#define _2PI 6.283185307f
#define _3PI_2 4.712388980f

#define CW 1
#define CCW -1

#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2957795457f

#define BLDC_MAX_VOLTAGE 12
#define BLDC_MIN_VOLTAGE 0

#define BUFF_SIZE 4096 //UART buffer size



#endif /* INC_DEF_H_ */
