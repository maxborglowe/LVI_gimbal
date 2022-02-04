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

#define GPIO_ENC_X GPIO_PIN_4
#define GPIO_ENC_Y GPIO_PIN_5
#define GPIO_ENC_Z GPIO_PIN_6

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define amt_encoders 3

extern uint8_t buff[512];
extern uint16_t curr_angle[amt_encoders];
extern float curr_angle_map[amt_encoders];
extern uint16_t zero_pos[amt_encoders];
extern float zero_pos_map[amt_encoders];
float angle[amt_encoders];

void as5048a_setZero(uint16_t arg_pos, uint8_t axis);
void as5048a_init(void);
void as5048a_getAllAngles(void);
int as5048a_getRotation(uint16_t ss);
float as5048a_readToAngle(uint16_t angle);
float as5048a_normalize(float angle);
uint16_t as5048a_getRawRotation(uint16_t ss);
uint16_t as5048a_read_angle(uint16_t ss, uint16_t reg);
uint8_t calcEvenParity(uint16_t value);

const char* getEncoderName(uint8_t enc);

#endif /* INC_AS5048A_H_ */
