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

#ifndef INC_DEF_H_
#define INC_DEF_H_

/*Configuration vars
 * Set these to configure what is read, written, printed, etc. in the while loop*/
//######################################
#define USE_SIM 0
#define USE_ICM20602 0
#define USE_BMI270 1
#define USE_AS5048A 0
#define USE_DRV8313 0
#define USE_PRINT 1
#define USE_IMU_VIS 1
#define ADC_DMA 0
#define ADC_INT 1
#define ADC_TYPE ADC_DMA
//######################################

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define AMT_MOTORS 3
#define CH_PER_MOTOR 3
#define ADC_CHANNELS AMT_MOTORS * CH_PER_MOTOR

#define BUFF_SIZE 2048


#endif /* INC_DEF_H_ */
