/*
 * FOC.c
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#include <stdio.h>
#include <foc.h>

float cos_val, sin_val;

float sqrtApprox(float number) {    //low in fat
	int32_t i;
	float y;
	// float x;
	// const float f = 1.5F; // better precision

	// x = number * 0.5F;
	y = number;
	i = *(int32_t*) &y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float*) &i;
	// y = y * ( f - ( x * y * y ) ); // better precision
	return number * y;
}

// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle) {
	float a = fmod(angle, _2PI);
	return a >= 0 ? a : (a + _2PI);
}

/**
 * @brief Do Park + Clarke-transforms on measured BLDC phase currents.
 */
void foc_ClarkePark(MotorDriver *driver) {
	/* Clarke-transform */
	/* Note: i_c not required, since KCL can be used --> i_a + i_b + i_c = 0*/
	float i_alpha = driver->i_a;
	float i_beta = (driver->i_a + 2 * driver->i_b) * _1_SQRT3;

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
	sin_val = FastTrigonometry_sin(driver->angle_electrical);
	cos_val = FastTrigonometry_cos(driver->angle_electrical);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);

	/* Park-transform */
	driver->i_d = i_alpha * cos_val + i_beta * sin_val;
	driver->i_q = i_beta * cos_val - i_alpha * sin_val;
}


/**
 * @brief Derive the electrical angle from rotor mechanical angle and amount of pole-pairs
 */
void foc_getElectricalAngle(MotorDriver *driver) {
	/* Derive electrical angle by multiplying mech. angle by pole-pair amount (and modulus 360˚)*/
	driver->angle_electrical = fmod(
			driver->pole_pairs * driver->angle - _PI, _2PI); /* + driver->offset*/
}

/**
 * @brief FOC main function.
 * Procedure:
 * 1. Fetch and transform mechanical angle to electrical + get rotor speed + get phase currents.
 * 2. Park + Clarke transform from 3-phase to dq-frame.
 * 3. Get direct and quadrature voltage references using PI-regulators.
 * 	  Speed regulator --> Vqref --> q-regulator
 * 	  Vdref = 0 --> d-regulator
 * 4. Do inverse Park-transform
 * 5. SVPWM algorithm
 */
void foc_update(MotorDriver *driver, float target) {

	/* down-sampling procedure */
	if (!driver->update_ctr) {
		/* Read mechanical angle */
		as5048a_getAngle(driver);
		/* Get the electrical angle*/
		foc_getElectricalAngle(driver);

		/* Clarke + Park-transform from 3-phase frame to 2-phase dq-frame*/
		foc_ClarkePark(driver);

		/* PI control: (Position -->) Velocity --> Direct + Quadrature*/
		as5048a_getVelocity(driver);
		foc_pi_control(driver, target);
	}
	driver->update_ctr = (driver->update_ctr + 1) % driver->update_goal;

	/* Inverse Park transformation */
	foc_invPark(driver);

	/* SVPWM */
	foc_setPhaseVoltage(driver, driver->V_d, driver->V_q);


}

/**
 * @brief All PI regulation required to get dq-reference voltages
 */
void foc_pi_control(MotorDriver *driver, float target) {



	/* Check which type of regulation should be used */
//	if (CONTROL_TYPE == CONTROL_POSITION){
		driver->velocity_target = PID_Update(&driver->pos_reg, target, driver->angle);
//	}
//	else if (CONTROL_TYPE == CONTROL_VELOCITY){
//		driver->velocity_target = target;
//	}

	/* Velocity regulation --> i_qref
	 * Note: Setpoint should be set by main function later*/
	float i_qref = PID_Update(&driver->speed_reg, driver->velocity_target,
			driver->velocity);


//	driver->i_d = lpf_exec(&driver->LPF_current_d, driver->i_d);
//	driver->i_q = lpf_exec(&driver->LPF_current_q, driver->i_q);

	/* current PI stuff */
	driver->V_d = PID_Update(&driver->d_reg, 0, driver->i_d);
	driver->V_q = PID_Update(&driver->q_reg, i_qref, driver->i_q);
}

/**
 * @brief Inverse Park-transform from static dq-frame to alternating alpha-beta frame.
 */
void foc_invPark(MotorDriver *driver) {
	driver->V_alpha = cos_val * driver->V_d - sin_val * driver->V_q;
	driver->V_beta = sin_val * driver->V_d + cos_val * driver->V_q;
}

/**
 * @brief Outputs three-phase PWM signal based on the input voltage vector components.
 * @param
 */
void foc_setPhaseVoltage(MotorDriver *driver, float V_d, float V_q) {



	float V_ref, a_duty = 0, b_duty = 0, c_duty = 0;

	V_ref = sqrtApprox(V_d * V_d + V_q * V_q);

	float T1, T2, T0;

	float m = _SQRT3 * V_ref * _1_Vdc;
//	float theta = fmod(atan2(V_beta, V_alpha) + _2PI, _2PI);


	float theta = _normalizeAngle(driver->angle_electrical + FastTrigonometry_atan2(V_q, V_d));


	/*Sector selection*/
	uint8_t sector = theta * _3_PI + 1;

	/* Duty time calculation */
	T1 = m * FastTrigonometry_sin(sector * _PI_3 - theta);
	T2 = m * FastTrigonometry_sin(theta - (sector - 1) * _PI_3);
	T0 = 1 - T1 - T2;

	switch (sector) {
		case 1:
			a_duty = T1 + T2 + T0 * 0.5;
			b_duty = T2 + T0 * 0.5;
			c_duty = T0 * 0.5;
			break;
		case 2:
			a_duty = T1 + T0 * 0.5;
			b_duty = T1 + T2 + T0 * 0.5;
			c_duty = T0 * 0.5;
			break;
		case 3:
			a_duty = T0 * 0.5;
			b_duty = T1 + T2 + T0 * 0.5;
			c_duty = T2 + T0 * 0.5;
			break;
		case 4:
			a_duty = T0 * 0.5;
			b_duty = T1 + T0 * 0.5;
			c_duty = T1 + T2 + T0 * 0.5;
			break;
		case 5:
			a_duty = T2 + T0 * 0.5;
			b_duty = T0 * 0.5;
			c_duty = T1 + T2 + T0 * 0.5;
			break;
		case 6:
			a_duty = T1 + T2 + T0 * 0.5;
			b_duty = T0 * 0.5;
			c_duty = T1 + T0 * 0.5;
			break;
	}

	drv8313_setPWM(driver, driver->timer->Instance, a_duty, b_duty, c_duty);
}

