/*
 * FOC.h
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <LowPassFilter.h>
#include "def.h"
#include "pid.h"
#include "DRV8313.h"
#include "AS5048A.h"
#include <math.h>
#include "time_utils.h"
#include "LowPassFilter.h"

#define V_dc 	BLDC_MAX_VOLTAGE
#define _1_Vdc 	1/BLDC_MAX_VOLTAGE



void foc_ClarkePark(MotorDriver *driver);
void foc_setPhaseVoltage3(MotorDriver *driver, float V_qref, float V_dref, float angle_el);
void foc_setPhaseVoltage2(MotorDriver *driver);
void foc_setPhaseVoltage(MotorDriver *driver, float V_alpha, float V_beta);
void foc_setPhaseVoltage_PI(MotorDriver *driver);
void foc_alignSensorToElectric(MotorDriver *driver);
void foc_getElectricalAngle(MotorDriver *driver);
void foc_alignSensorAbsoluteZero(MotorDriver *driver);

void foc_pi_control(MotorDriver *driver, float target);
void foc_pi_control2(MotorDriver *driver, float target);
void foc_invPark(MotorDriver *driver);
void foc_invPark2(void);
void foc_update(MotorDriver *driver, float target);

float sin_fast(float a);
float cos_fast(float a);
float sqrtApprox(float number);

#endif /* INC_FOC_H_ */
