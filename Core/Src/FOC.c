/*
 * FOC.c
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#include "FOC.h"

float cos_val, sin_val;
uint16_t move_counter = 0;
uint16_t move_exec = 5;

//const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

float sqrtApprox(float number) {//low in fat
  int32_t i;
  float y;
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  y = number;
  i = *(int32_t*) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = *(float*) &i;
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}

// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

void foc_init(MotorDriver *driver){
//	foc_alignSensorToElectric(driver);
}

/**
 * @brief Do Park + Clarke-transforms on measured BLDC phase currents.
 */
void foc_ClarkePark(MotorDriver *driver) {
	/* Clarke-transform */
	/* Note: i_c not required, since KCL can be used --> i_a + i_b + i_c = 0*/
	float i_alpha = driver->i_a;
	float i_beta = (driver->i_a + 2*driver->i_b)*_1_SQRT3;


	sin_val = sin(driver->angle_electrical);
	cos_val = cos(driver->angle_electrical);

	/* Park-transform */
	driver->i_d = i_alpha*cos_val + i_beta*sin_val;
	driver->i_q = i_beta*cos_val - i_alpha*sin_val;
}

//void foc_alignSensorToElectric(MotorDriver *driver){
//
//	as5048a_getAngle(driver);
//	float start_angle = driver->angle;
//
//	for (int i = 0; i <=500; i++ ) {
//		float angle = _3PI_2 + _2PI * i / 500.0f;
//		foc_setPhaseVoltage(driver, V_dc, 0, angle*RAD_TO_DEG);
//	}
//
//	HAL_Delay(20);
//
//	as5048a_getAngle(driver);
//	float mid_angle = driver->angle;
//
//	for (int i = 500; i >=0; i-- ) {
//		float angle = _3PI_2 + _2PI * i / 500.0f ;
//		foc_setPhaseVoltage(driver, V_dc, 0, angle*RAD_TO_DEG);
//	}
//
//	HAL_Delay(20);
//
//
//	//may not be necessary
//	if(start_angle == mid_angle){
//		driver->direction = 0;
//	}
//	else if(start_angle < mid_angle){
//		driver->direction = CCW;
//	}
//	else{
//		driver->direction = CW;
//	}
//
//
//	/* Align rotor flux */
//
//
//}

/**
 * @brief Derive the electrical angle from rotor mechanical angle and amount of pole-pairs
 */
void foc_getElectricalAngle(MotorDriver *driver){
	float offset = 0*DEG_TO_RAD;
	/* Derive electrical angle by multiplying mech. angle by pole-pair amount (and modulus 360˚)*/
	driver->angle_electrical = fmod(driver->pole_pairs * driver->angle + offset - _PI, _2PI);
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
void foc_update(MotorDriver *driver, float target){

	if(!move_counter){
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
	move_counter = (move_counter + 1) % move_exec;


	/* SVPWM */
	foc_setPhaseVoltage(driver, driver->V_d, driver->V_q);

}

/**
 * @brief All PI regulation required to get dq-reference voltages
 */
void foc_pi_control(MotorDriver *driver, float target){

	/* Check which type of regulation should be used */
	if(CONTROL_TYPE == CONTROL_POSITION) driver->velocity_target = PID_Update(&driver->pos_reg, target, driver->angle);
	else if(CONTROL_TYPE == CONTROL_VELOCITY) driver->velocity_target = target;

	/* Velocity regulation --> i_qref
	 * Note: Setpoint should be set by main function later*/
	float i_qref = PID_Update(&driver->speed_reg, driver->velocity_target, driver->velocity);

	driver->i_d = lpf_exec(&driver->LPF_current_d, driver->i_d);
	driver->i_q = lpf_exec(&driver->LPF_current_q, driver->i_q);
	/* current PI stuff */
	driver->V_d = PID_Update(&driver->d_reg, 0, driver->i_d);
	driver->V_q = PID_Update(&driver->q_reg, i_qref, driver->i_q);
}

/**
 * @brief Inverse Park-transform from static dq-frame to alternating alpha-beta frame.
 */
void foc_invPark(MotorDriver *driver){

	driver->V_alpha = cos_val * driver->V_d - sin_val * driver->V_q;
	driver->V_beta = sin_val * driver->V_d + cos_val * driver->V_q;
}

void foc_setPhaseVoltage(MotorDriver *driver, float V_alpha, float V_beta){


	float V_ref, a_duty = 0, b_duty = 0, c_duty = 0;


	V_ref = sqrtApprox(V_alpha * V_alpha + V_beta * V_beta);

	float T1, T2, T0;

	float m = _SQRT3 * V_ref * _1_Vdc;
//	float theta = fmod(atan2(V_beta, V_alpha) + _2PI, _2PI);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, SET);
	float theta = _normalizeAngle(driver->angle_electrical + atan2(V_beta, V_alpha));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, !SET);

	/*Sector selection*/
	uint8_t sector = theta * _3_PI + 1;


	/* Duty time calculation */
	T1 = m * sin(sector * _PI_3 - theta);
	T2 = m * sin(theta - (sector - 1) * _PI_3);
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


	drv8313_setPWM(driver, driver->timer->Instance, a_duty, b_duty, c_duty);
}

