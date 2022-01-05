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
#include "pid.h"

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

	float i_a, i_b, i_d, i_q; /* Current sensor variables */
	PID d_reg, q_reg, speed_reg;
	float current_setpoint;
	float V_q, V_d;

	TIM_HandleTypeDef *timer; /* PWM timer handle */

	uint8_t pwm_ch1, pwm_ch2, pwm_ch3; /* Externally set PWM channels */

	uint16_t curr_angle, zero_pos;
	float curr_angle_map, zero_pos_map, angle, prev_angle; /* Current motor angle */
	float speed_rpm;

	/* Offset angle used when determining phase voltages */
	float angle_electrical, zero_angle_electrical;

	/* Amount of pole pairs in the motor*/
	uint16_t pole_pairs;
	/* In which direction the sensor is turned when driving the motor */
	int8_t direction;
} MotorDriver;

uint8_t drv8313_init(MotorDriver *driver, TIM_HandleTypeDef *htim);
void drv8313_setPWM(MotorDriver *driver, float duty_a, float duty_b, float duty_c);
void drv8313_setDutyLookup(MotorDriver *driver);

#endif /* INC_DRV8313_H_ */
