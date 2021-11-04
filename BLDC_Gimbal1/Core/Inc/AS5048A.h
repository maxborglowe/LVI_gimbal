/*
 * spi.h
 *
 *  Created on: Nov 3, 2021
 *      Author: maxborglowe
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define CMD_READ 0x4000 //PARITY = 0, RW = R
#define REG_ANGLE 0x3FFF

extern SPI_HandleTypeDef hspi1;

void as5048a_setZero(uint16_t arg_pos);
int as5048a_getRotation(uint16_t ss);
float as5048a_readToAngle(uint16_t angle);
float as5048a_normalize(float angle);
uint16_t as5048a_getRawRotation(uint16_t ss);
uint16_t as5048a_read_angle(uint16_t ss, uint16_t reg);
uint8_t calcEvenParity(uint16_t value);

#endif /* INC_AS5048A_H_ */
