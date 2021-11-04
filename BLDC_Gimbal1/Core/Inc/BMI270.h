/*
 * BMI270.h
 *
 *  Created on: Nov 1, 2021
 *      Author: maxborglowe
 */

#ifndef INC_BMI270_H_
#define INC_BMI270_H_


#include "stm32f4xx_hal.h"



//Default address. If this address is used, SDO must be pulled to GND.
#define BMI270_ADDR 0b1101000<<1

//Expected response when reading REG_CHIP_ID
#define CHIP_ID 0x24

#define REG_CHIP_ID 0x00
#define REG_INIT_CTRL 0x59
#define REG_INIT_DATA 0x5E
#define REG_PWR_CONF 0x7C

extern uint8_t buff[256];
extern UART_HandleTypeDef huart2;

void bmi270_init(void);



#endif /* INC_BMI270_H_ */
