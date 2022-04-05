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
#include "LowPassFilter.h"

extern ADC_HandleTypeDef hadc1;

#define REGULATE_D 0
#define REGULATE_Q 1
#define REGULATE_SPEED 2
#define REGULATE_POS 3
#define REGULATE_IMU 4

typedef struct MotorDriver{
	uint16_t PIN_ENC; /* Chip select pin for encoder */
	uint16_t PIN_nFAULT; /* Fault detector pin for BLDC driver*/
	uint16_t update_ctr; /* Counter variable to keep track of when FOC should update */
	uint16_t update_goal; /* The value to be reached before executing FOC update */

	float i_a, i_b, i_d, i_q; /* Current parameters */
	uint8_t i_a_ch, i_b_ch; /* Current sensor ADC channels*/
	float V_q, V_d, V_alpha, V_beta; /* Voltage parameters*/
	float offset; /* electro-mechanical offset in BLDC */


	PID d_reg, q_reg, speed_reg, pos_reg, imu_reg; /* PID regulators */
	LPF LPF_current_d, LPF_current_q, LPF_velocity, LPF_angle, LPF_imu;

	TIM_HandleTypeDef *timer; /* PWM timer handle */
	uint8_t pwm_ch1, pwm_ch2, pwm_ch3; /* Externally set PWM channels */
	uint16_t pwm_period; /* */

	float curr_angle, zero_pos;
	float curr_angle_map, zero_pos_map, angle, prev_angle; /* Current motor angle */

	/* Electrical angle derived from mech. angle & pole-pars */
	float angle_electrical; // zero_angle_electrical;

	uint32_t prev_timestamp_us; /* previous microsecond tick for RPM calculation */
	float velocity;
	float velocity_target;

	/* Amount of pole pairs in the motor*/
	uint16_t pole_pairs;
} MotorDriver;

uint8_t drv8313_init(MotorDriver *driver, TIM_HandleTypeDef *htim);
void drv8313_setPWM(MotorDriver *driver, TIM_TypeDef *tim_instance, float duty_a, float duty_b, float duty_c);
//void drv8313_setPID(MotorDriver *driver, uint8_t regulation_sel, float P, float I, float D);

#endif /* INC_DRV8313_H_ */
