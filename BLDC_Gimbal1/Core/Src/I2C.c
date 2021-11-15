/*
 * I2C.c
 *
 *  Created on: Nov 1, 2021
 *      Author: maxborglowe
 */

#include "I2C.h"

/*
 * Write 1 byte of data using I2C.
 * @param the 7-bit I2C-address
 * @param the 8-bit register to which data is written
 * @param this is what is written into the chosen 8-bit register
 */
HAL_StatusTypeDef i2c_write(uint8_t ADDR, uint8_t reg, uint8_t config) {
	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t) ADDR << 1, (uint16_t) (reg),
			0x01, &config, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}
	return HAL_OK;
}

/*
 * Write 1 byte of data using I2C.
 * @param the 7-bit I2C-address
 * @param the 8-bit register from which data is read
 * @param input variable onto which data from the IMU is written (NOTE: input is &somevar)
 */
HAL_StatusTypeDef i2c_read_8(uint8_t ADDR, uint8_t reg, uint8_t *read_var) {
	uint8_t data;

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) (ADDR << 1 | 0x00),
			(uint16_t) (reg), 0x01, &data, 1, 500);
	if (status != HAL_OK) {
		return status;
	}

	*read_var = data;
	return HAL_OK;
}

/*
 * Write 1 byte of data using I2C.
 * @param the 7-bit I2C-address
 * @param the LOW 8-bit register from which data is read
 * @param the HIGH 8-bit register from which data is read
 * @param input address onto which data from the IMU is written (NOTE: input is &somevar)
 */
HAL_StatusTypeDef i2c_read_16(uint8_t ADDR, uint8_t lo_reg, uint8_t hi_reg,
		int16_t *read_var) {
	uint8_t data[2];

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) (ADDR << 1 | 0x01),
			(uint16_t) (lo_reg), 0x01, &(data[0]), 1, 500);
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) (ADDR << 1 | 0x01),
			(uint16_t) (hi_reg), 0x01, &(data[1]), 1, 500);
	if (status != HAL_OK) {
		return status;
	}

	*read_var = (data[1] << 8 | data[0]);
	return HAL_OK;
}

uint8_t i2c_master_read(uint8_t ADDR, uint8_t reg, uint8_t *read_var,
		uint16_t amt) {
	uint16_t DevAddr = (ADDR << 1);
	HAL_I2C_Master_Transmit(&hi2c1, DevAddr, &reg, 1, 500);
	HAL_I2C_Master_Receive(&hi2c1, DevAddr, read_var, amt, 500);

	return 0;
}
