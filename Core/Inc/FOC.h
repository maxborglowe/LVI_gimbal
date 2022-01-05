/*
 * FOC.h
 *
 *  Created on: Nov 24, 2021
 *      Author: maxborglowe
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include "def.h"
#include "pid.h"
#include "DRV8313.h"
#include "AS5048A.h"
#include <math.h>
#include "Quaternions.h"

#define V_dc BLDC_MAX_VOLTAGE

void foc_ClarkePark(MotorDriver *driver);
void foc_setPhaseVoltage(MotorDriver *driver, float V_qref, float V_dref, float angle_el);
void foc_setPhaseVoltage_PI(MotorDriver *driver);
void foc_alignToRotor(MotorDriver *driver);
float foc_getElectricalAngle(MotorDriver *driver);

void foc_motion(MotorDriver *driver, float target);
void foc_update(MotorDriver *driver);

#endif /* INC_FOC_H_ */
