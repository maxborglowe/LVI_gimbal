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
#include <stdint.h>

#ifndef INC_DEF_H_
#define INC_DEF_H_

/* Configuration vars
 * Set these to configure what is read, written, printed, etc. in the while loop
 * 0 = disable function, 1 = enable function*/
//######################################
#define USE_BMI270 1
#define USE_AS5048A 1
#define USE_DRV8313 1
#define USE_PRINT 0
#define USE_IMU_VIS 0
#define USE_IMU_CONTROL 0 /*0 = Joystick control. 1 = IMU control*/
//######################################

/* GPIO stuff */
//######################################
#define PINBUS_ENC GPIOB
#define PIN_ENC_X GPIO_PIN_4
#define PIN_ENC_Y GPIO_PIN_5
#define PIN_ENC_Z GPIO_PIN_6

#define PINBUS_BMI270 GPIOB
#define PIN_BMI270_CS GPIO_PIN_0

#define PINBUS_DRV8313 GPIOB
#define PIN_nSLEEP GPIO_PIN_13
#define PIN_nFAULT_X GPIO_PIN_14
#define PIN_nFAULT_Y GPIO_PIN_15
#define PIN_nFAULT_Z GPIO_PIN_12


/* IMU stuff */
//######################################
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
//######################################

/* Motor & CurrentSense stuff*/
//######################################
#define BLDC_MAX_VOLTAGE 12.0f
#define BLDC_MIN_VOLTAGE 0.0f
#define BLDC_PHASE_RESISTANCE 5.275f /* Measured line-to-line resistance divided by 2 */

#define CONTROL_TYPE 1
#define CONTROL_VELOCITY 0
#define CONTROL_POSITION 1

#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_DIGITAL_VALUE DIGITAL_MAX_8BIT
#define CURRENT_SENSE_RESISTANCE 0.226f
#define CURRENT_SENSE_GAIN 20.0f //INAx181A1 gain = 20x

#define DIGITAL_MAX_12BIT (4096-1)
#define DIGITAL_MAX_10BIT (1024-1)
#define DIGITAL_MAX_8BIT (256-1)

/* Math stuff */
//######################################
#define _sign(a) (((a) < 0)? -1: ((a) > 0))
#define _round(x) ((x)>=0? (long)((x)+0.5f): (long)((x)-0.5f))
#define _clamp(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define MACLAURIN_PRECISION 2 /* How many iterations to run Maclaurin expressions. Higher value = better resolution*/

#define _1_SQRT2 0.7071067
#define _1_SQRT3 0.5773503
#define _SQRT3 1.7320501
#define _rl_e_j2PI_3 -0.5f
#define _im_e_j2PI_3 0.866025404f
#define _rl_e_j4PI_3 -0.5f
#define _im_e_j4PI_3 -0.866025404f
#define _1_3 0.33333333333f
#define _2_3 0.66666666667f

#define _2PI_3 _PI*_2_3

#define _3_PI 	0.95492965855f
#define _PI_3 	1.0471975512f
#define _PI_2 	1.57079632679f
#define _PI 	3.14159265359f
#define _3PI_2 	4.71238898038f
#define _2PI 	6.28318530718f

#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2957795457f

#define BUFF_SIZE 4096 			/* UART buffer size*/

#endif /* INC_DEF_H_ */
