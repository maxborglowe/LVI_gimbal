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
#include <math.h>
#include "def.h"
#include "DRV8313.h"


#define CMD_READ 0x4000 //PARITY = 0, RW = R
#define REG_ANGLE 0x3FFF

#define PINBUS_ENC GPIOB
#define PIN_ENC_X GPIO_PIN_4
#define PIN_ENC_Y GPIO_PIN_5
#define PIN_ENC_Z GPIO_PIN_6

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

extern char* buff[BUFF_SIZE];
extern uint16_t curr_angle[AMT_MOTORS];
extern float curr_angle_map[AMT_MOTORS];
extern uint16_t zero_pos[AMT_MOTORS];
extern float zero_pos_map[AMT_MOTORS];
float angle[AMT_MOTORS];

void as5048a_setZeroArg(MotorDriver *driver, uint16_t arg_pos);
void as5048a_setZero(MotorDriver *driver);
void as5048a_init(MotorDriver *driver);
void as5048a_getAllAngles(void);
void as5048a_getAngle(MotorDriver *driver);

int as5048a_getRotation(uint16_t ss);
float as5048a_readToAngle(uint16_t angle);
float as5048a_normalize(float angle);
uint16_t as5048a_getRawRotation(uint16_t ss);
uint16_t as5048a_read_angle(uint16_t ss, uint16_t reg);
uint8_t calcEvenParity(uint16_t value);

const char* getEncoderName(uint8_t enc);

#endif /* INC_AS5048A_H_ */
