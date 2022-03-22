/*
 * spi.h
 *
 *  Created on: Nov 3, 2021
 *      Author: maxborglowe
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include <stdio.h>
#include <math.h>
#include "def.h"
#include "DRV8313.h"
#include "time_utils.h"

#define CMD_READ 0x4000 //PARITY = 0, RW = R
#define CMD_WRITE 0x0000 //PARITY = 0, RW = W
#define REG_ANGLE 0x3FFF
#define REG_OTP_REGISTER_ZERO_POSITION_HI 0x0016
#define REG_OTP_REGISTER_ZERO_POSITION_LOW_6_LSBS 0x0017

#define _1_16384 1/16384.0f /* Used to convert bits to angle */

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

uint16_t as5048a_read(uint16_t ss, uint16_t reg);

void as5048a_setZeroArg(MotorDriver *driver, float arg_pos);
void as5048a_setZero(MotorDriver *driver);
void as5048a_init(MotorDriver *driver);
void as5048a_getAngle(MotorDriver *driver);
void as5048a_getVelocity(MotorDriver *driver);

int as5048a_getRotation(uint16_t ss);
float as5048a_readToAngle(uint16_t angle);
float as5048a_normalize(float angle);
uint16_t as5048a_getRawRotation(uint16_t ss);
uint16_t as5048a_read_angle(uint16_t ss, uint16_t reg);
uint8_t calcEvenParity(uint16_t value);

#endif /* INC_AS5048A_H_ */
