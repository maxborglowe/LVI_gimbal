/*
 * DRV8313.h
 *
 *  Created on: Nov 17, 2021
 *      Author: maxborglowe
 */

#ifndef INC_DRV8313_H_
#define INC_DRV8313_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "def.h"

extern ADC_HandleTypeDef hadc1;

#define PINBUS_DRV8313 GPIOB
#define PIN_nSLEEP GPIO_PIN_13
#define PIN_nFAULT GPIO_PIN_14

/* SVPWM definitions */
#define PWM_PERIOD 1024
#define DIV1_3 0.333333
#define DIV2_3 0.666667

typedef struct MotorDriver{
	uint16_t PIN_ENC; /* Chip select pin for encoder */

	uint16_t sense1, sense2, sense3; /* Current sensors */

	TIM_HandleTypeDef *timer; /* PWM timer handle */

	uint8_t pwm_ch1, pwm_ch2, pwm_ch3; /* Externally set PWM channels */

	uint16_t curr_angle, zero_pos;
	float curr_angle_map, zero_pos_map, angle; /* Current motor angle */
} MotorDriver;

uint8_t drv8313_init(MotorDriver *driver, TIM_HandleTypeDef *htim);
void drv8313_sense(MotorDriver *driver);
void drv8313_setDuty(MotorDriver *driver, float duty);
void drv8313_setDutyLookup(MotorDriver *driver);

#endif /* INC_DRV8313_H_ */
