/*
 * I2C.h
 *
 *  Created on: Nov 1, 2021
 *      Author: maxborglowe
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern HAL_StatusTypeDef status;

HAL_StatusTypeDef i2c_write(uint8_t ADDR, uint8_t reg, uint8_t config);
HAL_StatusTypeDef i2c_read_8(uint8_t ADDR, uint8_t reg, uint8_t *read_var);
HAL_StatusTypeDef i2c_read_16(uint8_t ADDR, uint8_t lo_reg, uint8_t hi_reg,
		int16_t *read_var);

#endif /* INC_I2C_H_ */
