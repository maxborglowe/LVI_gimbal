/*
 * DRV8313.c
 *
 *  Created on: Nov 17, 2021
 *      Author: maxborglowe
 */

#include "DRV8313.h"

/*!
 * @brief Initialize the DRV8313.
 * @param Driver into which references to timers etc. will be stored
 * @param Timer to control the PWM signals.
 * Procedure:
 * 1: Set nRESET and nSLEEP to inactive HIGH to enable the three phase H-bridge.
 * 2: Read the nFAULT pin. If the pin is active HIGH, the initialization stops. If inactive LOW, the procedure continues.
 * 3: Pass references to timers and timer channels so that the PWM function of each timer may be used.
 * 4: Start PWM.
 */
uint8_t drv8313_init(MotorDriver *driver, TIM_HandleTypeDef *htim) {
	HAL_GPIO_WritePin(PINBUS_DRV8313, PIN_nSLEEP, GPIO_PIN_SET); /* Enable the unit by setting nRESET + nSLEEP to HIGH*/
	HAL_Delay(1); //Misread prevention delay.
	if (!HAL_GPIO_ReadPin(PINBUS_DRV8313, driver->PIN_nFAULT)) {
		return 0;
	}

	driver->timer = htim;

	driver->pwm_ch1 = TIM_CHANNEL_1;
	driver->pwm_ch2 = TIM_CHANNEL_2;
	driver->pwm_ch3 = TIM_CHANNEL_3;

	/* PID config */
	PID_Init(&driver->d_reg);
	PID_Init(&driver->q_reg);
	PID_Init(&driver->speed_reg);
	PID_Init(&driver->pos_reg);

	driver->d_reg.lim_min = -BLDC_MAX_VOLTAGE;
	driver->d_reg.lim_max = BLDC_MAX_VOLTAGE;
	driver->q_reg.lim_min = -BLDC_MAX_VOLTAGE;
	driver->q_reg.lim_max = BLDC_MAX_VOLTAGE;
	driver->speed_reg.lim_min = -BLDC_MAX_VOLTAGE/BLDC_PHASE_RESISTANCE;
	driver->speed_reg.lim_max = BLDC_MAX_VOLTAGE/BLDC_PHASE_RESISTANCE;
//	driver->pos_reg.lim_min = -6000; 		/* ˚/s */
//	driver->pos_reg.lim_max = 6000;		/* ˚/s */
	driver->pos_reg.lim_min = -52.35; 		/* rad/s */
	driver->pos_reg.lim_max = 52.35;		/* rad/s */

	/* d-regulator */
	driver->d_reg.Kp = 1.0f;
	driver->d_reg.Ki = 0.0f;
	driver->d_reg.Kd = 0.0f;
	/* q-regulator */
	driver->q_reg.Kp = 1.0f;
	driver->q_reg.Ki = 0.0f;
	driver->q_reg.Kd = 0.0f;

	/* speed regulator */
	driver->speed_reg.Kp = 0.25f;
	driver->speed_reg.Ki = 10.0f;
	driver->speed_reg.Kd = 0.0f;
	/**/

	/* position regulator */
	driver->pos_reg.Kp = 20.0f;
	driver->pos_reg.Ki = 0.0f;
	driver->pos_reg.Kd = 0.25f;

	/* LPF config */
	lpf_init(&driver->LPF_current_d, 0.0025f);
	lpf_init(&driver->LPF_current_q, 0.0025f);
	lpf_init(&driver->LPF_velocity, 0.05f);
	lpf_init(&driver->LPF_angle, 0.005f);

	HAL_TIM_PWM_Start(driver->timer, driver->pwm_ch1);
	HAL_TIM_PWM_Start(driver->timer, driver->pwm_ch2);
	HAL_TIM_PWM_Start(driver->timer, driver->pwm_ch3);

	/* Calculate PWM period */
	driver->pwm_period = driver->timer->Init.Period + 1;

	return 1;

}

/*!
 * @brief Set the PWM duty cycle on each phase of a BLDC
 * @param BLDC pointer
 * @param duty cycle on phase a
 * @param duty cycle on phase b
 * @param duty cycle on phase c
 */
void drv8313_setPWM(MotorDriver *driver, TIM_TypeDef *tim_instance, float duty_a, float duty_b, float duty_c) {

	/* Wait for PWM period to finish before setting new duty period
	 * Note: May be unnecessary, but used as a safety measure for now */
//	while(tim_instance->CNT != 0){
//	}

	tim_instance->CCR1 = duty_a * driver->pwm_period;
	tim_instance->CCR2 = duty_b * driver->pwm_period;
	tim_instance->CCR3 = duty_c * driver->pwm_period;
}



