/*
 * FOC.c
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#include "FOC.h"



void foc_ClarkePark(MotorDriver *driver) {
	/* Clarke-transform */
	/* Note: i_c not required, since KCL can be used --> i_a + i_b + i_c = 0*/
	float i_alpha = driver->i_a;
	float i_beta = (driver->i_a + 2*driver->i_b)*_1_SQRT3;

	float cos_val = cos(driver->angle_electrical*DEG_TO_RAD), sin_val = sin(driver->angle_electrical*DEG_TO_RAD);

	/* Park-transform */
	driver->i_d = i_alpha*cos_val + i_beta*sin_val;
	driver->i_q = i_beta*cos_val - i_alpha*sin_val;
}

void foc_alignToRotor(MotorDriver *driver){

	for (int i = 0; i <=500; i++ ) {
			  float angle = _3PI_2 + _2PI * i / 500.0f;
			  foc_setPhaseVoltage(driver, V_dc, 0, angle);
			}

	HAL_Delay(2);

	as5048a_getAngle(driver);
	float mid_angle = driver->angle;

	for (int i = 500; i >=0; i-- ) {
		  float angle = _3PI_2 + _2PI * i / 500.0f ;
		  foc_setPhaseVoltage(driver, V_dc, 0, angle);
	}

	HAL_Delay(2);
	as5048a_getAngle(driver);
	float end_angle = driver->angle;

//	if(mid_angle == end_angle){
//		driver->direction = 0;
//	}
//	else if(mid_angle < end_angle){
//		driver->direction = CCW;
//	}
//	else{
//		driver->direction = CW;
//	}

	/* Set zero electric angle*/
	foc_setPhaseVoltage(driver, V_dc, 0, _3PI_2);
	HAL_Delay(500);

	driver->zero_angle_electrical = 0;
	as5048a_getAngle(driver);
	driver->zero_angle_electrical = foc_getElectricalAngle(driver);

	HAL_Delay(200);
}

float foc_getElectricalAngle(MotorDriver *driver){
	return fmod((float)(driver->direction * driver->pole_pairs) * driver->angle - driver->zero_angle_electrical + 360, 360);
}

/**
 * @brief Convert angular input to 0 - 360 deg
 * @param Input angle (in degrees)
 */
float foc_normalizeAngle(float angle){
  return fmod(angle + 360, 360);
}

/**
 * @brief
 */
void foc_update(MotorDriver *driver){
	foc_motion(driver, 500);

	/* Get the electrical angle*/
	as5048a_getAngle(driver);
	driver->angle_electrical = foc_getElectricalAngle(driver);

	foc_ClarkePark(driver);

	/* PI stuff */
	driver->V_d = PID_Update(&driver->d_reg, 0, driver->i_d);
	driver->V_q = PID_Update(&driver->q_reg, driver->current_setpoint, driver->i_q);

	foc_setPhaseVoltage(driver, driver->V_q, driver->V_d, driver->angle_electrical);
}

void foc_motion(MotorDriver *driver, float target){
	driver->current_setpoint = PID_Update(&driver->speed_reg, target, driver->speed_rpm);
	driver->V_q = driver->current_setpoint;
	driver->V_d = 0;
}

void foc_setPhaseVoltage(MotorDriver *driver, float V_qref, float V_dref, float angle_el){
	float V_ref, a_duty, b_duty, c_duty;


	V_ref = sqrt(V_dref*V_dref + V_qref*V_qref);


	float T1, T2, T0;

	float m = _1_SQRT3*V_ref/V_dc;
	float theta = fmod((atan2(V_qref, V_dref) + angle_el)*RAD_TO_DEG + 360, 360); //RAD_TO_DEG*_normalizeAngle(angle_el + atan2(V_qref, V_dref));

	/*Sector selection*/
	uint8_t sector = theta/60 + 1;
	//theta = fmod(theta, 60);

	/* Duty time calculation */
	T1 = m*sin(sector*_1PI_3 - theta*DEG_TO_RAD);
	T2 = m*sin(theta*DEG_TO_RAD - (sector - 1) * _1PI_3);
	T0 = 1 - T1 - T2;

		switch (sector) {
			case 1:
				a_duty = T1 + T2 + T0*0.5;
				b_duty = T2 + T0*0.5;
				c_duty = T0*0.5;
				break;
			case 2:
				a_duty = T1 + T0*0.5;
				b_duty = T1 + T2 + T0*0.5;
				c_duty = T0*0.5;
				break;
			case 3:
				a_duty = T0*0.5;
				b_duty = T1 + T2 + T0*0.5;
				c_duty = T2 + T0*0.5;
				break;
			case 4:
				a_duty = T0*0.5;
				b_duty = T1 + T0*0.5;
				c_duty = T1 + T2 + T0*0.5;
				break;
			case 5:
				a_duty = T2 + T0*0.5;
				b_duty = T0*0.5;
				c_duty = T1 + T2 + T0*0.5;
				break;
			case 6:
				a_duty = T1 + T2 + T0*0.5;
				b_duty = T0*0.5;
				c_duty = T1 + T0*0.5;
				break;
		}

	drv8313_setPWM(driver, a_duty, b_duty, c_duty);
}


/*!
 * @brief PI control and inverse Park- and Clarke-transform
 */
//void foc_setPhaseVoltage_PI(MotorDriver *driver){
//	float V_dref, V_qref, i_qref, V_ref, a_duty, b_duty, c_duty;
//
//	i_qref = PID_Update(&driver->speed_reg, 400, driver->speed_rpm);
//	V_dref = PID_Update(&driver->d_reg, 0, driver->i_d);
//	V_qref = PID_Update(&driver->q_reg, i_qref, driver->i_q);
//
//	V_ref = sqrt(V_dref*V_dref + V_qref*V_qref);
//
//
//	float T1, T2, T0;
//
//	float m = _SQRT3 * V_ref/V_dc;
//	float theta = fmod((atan2(V_qref, V_dref) + driver->angle)*RAD_TO_DEG + 360, 360);
//
//	/*Sector selection*/
//	uint8_t sector = theta/60 + 1;
//
//	/* Duty time calculation */
//	T1 = sin(sector*_1PI_3 - theta*DEG_TO_RAD);
//	T2 = sin(theta*DEG_TO_RAD - (sector - 1) * _1PI_3);
//	T0 = 1 - T1 - T2;
//
//		switch (sector) {
//			case 1:
//				a_duty = T1 + T2 + T0*0.5;
//				b_duty = T2 + T0*0.5;
//				c_duty = T0*0.5;
//				break;
//			case 2:
//				a_duty = T1 + T0*0.5;
//				b_duty = T1 + T2 + T0*0.5;
//				c_duty = T0*0.5;
//				break;
//			case 3:
//				a_duty = T0*0.5;
//				b_duty = T1 + T2 + T0*0.5;
//				c_duty = T2 + T0*0.5;
//				break;
//			case 4:
//				a_duty = T0*0.5;
//				b_duty = T1 + T0*0.5;
//				c_duty = T1 + T2 + T0*0.5;
//				break;
//			case 5:
//				a_duty = T2 + T0*0.5;
//				b_duty = T0*0.5;
//				c_duty = T1 + T2 + T0*0.5;
//				break;
//			case 6:
//				a_duty = T1 + T2 + T0*0.5;
//				b_duty = T0*0.5;
//				c_duty = T1 + T0*0.5;
//				break;
//		}
//
//	drv8313_setPWM(driver, a_duty, b_duty, c_duty);
//}
