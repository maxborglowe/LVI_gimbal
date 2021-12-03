/*
 * spi.c
 *
 *  Created on: Nov 3, 2021
 *      Author: maxborglowe
 */

#include <AS5048A.h>


/*
 * @brief Read 16-bit data at selected register.
 * @param SPI slave select pin.
 * @param AS5048A register
 */
uint16_t as5048a_read(uint16_t ss, uint16_t reg) {
	uint8_t data[2];

	uint16_t cmd = CMD_READ | reg;
	cmd |= ((uint16_t) calcEvenParity(cmd) << 15);

	data[1] = cmd & 0xFF;
	data[0] = (cmd >> 8) & 0xFF;

	HAL_GPIO_WritePin(GPIOB, ss, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
	}
	HAL_GPIO_WritePin(GPIOB, ss, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOB, ss, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, (uint8_t*) &data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
	}
	HAL_GPIO_WritePin(GPIOB, ss, GPIO_PIN_SET);

	return (((data[0] & 0xFF) << 8) | (data[1] & 0xFF)) & ~0xC000; //wat
}

/*
 * @brief Get raw rotation as 16-bit value.
 * @param SPI slave select pin.
 */
uint16_t as5048a_getRawRotation(uint16_t ss) {
	return as5048a_read(ss, REG_ANGLE);
}

/*
 * @brief Initialize the SPI by setting all encoder pins high.
 */
void as5048a_init(MotorDriver *driver) {
	HAL_GPIO_WritePin(PINBUS_ENC, driver->PIN_ENC, GPIO_PIN_SET);

	driver->zero_pos = as5048a_getRawRotation(driver->PIN_ENC);
	driver->zero_pos_map = as5048a_readToAngle(driver->zero_pos);

//
//	HAL_GPIO_WritePin(GPIOB, PIN_ENC_X, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, PIN_ENC_Y, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, PIN_ENC_Z, GPIO_PIN_SET);
	HAL_Delay(1);
}

/*
 * @brief Normalized the input angle, meaning that the angle starts at 0˙ and ends at 360˙
 * @param unnormalized input angle
 */
float as5048a_normalize(float angle) {
	//angle += 180;
	angle = fmod(angle, 360);
	if (angle < 0) {
		angle += 360;
	}
	//angle -= 180;
	return angle;
}

/*
 * @brief Set the zero position to arbitrary value.
 * Use: When moving the camera by hand, you could make the zero position the same as the current angle
 * @param input angle
 */
void as5048a_setZeroArg(MotorDriver *driver, uint16_t arg_pos) {
	driver->zero_pos = arg_pos % 360;
}

/*
 * @brief Convert raw data from getRawRotation to angles in degrees.
 * @param Raw angular data input.
 */
float as5048a_readToAngle(uint16_t angle) {
	return 2 * ((float) angle * ((float) 360 / 16383) - 180);
}

uint8_t calcEvenParity(uint16_t value) {
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++) {
		if (value & 0x1) {
			cnt++;
		}

		value >>= 1;
	}
	return cnt & 0x1;
}

/*
 * @brief Fetches current angle on each encoder.
 */
//void as5048a_getAllAngles() {
//	for (uint8_t c = 0; c < AMT_MOTORS; c++) {
//		curr_angle[c] = as5048a_getRawRotation(PIN_ENC_X << c);
//		curr_angle_map[c] = as5048a_readToAngle(curr_angle[c]);
//
//		angle[c] = curr_angle_map[c] - zero_pos_map[c];
//		angle[c] = as5048a_normalize(angle[c]);
//
//		const char *enc = getEncoderName(c);
//
//		sprintf((char*) buff, "zero pos %s: %f\r\n"
//				"encoder %s: %.3f\r\n", enc, zero_pos_map[c], enc, angle[c]);
//		HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), 100);
//	}
//}

///*
// * @brief Fetches current angle on selected encoder axis.
// */
//uint8_t as5048_getAngle(uint8_t enc){
//	curr_angle[enc] = as5048a_getRawRotation(PIN_ENC_X << enc);
//	curr_angle_map[enc] = as5048a_readToAngle(curr_angle[enc]);
//
//	angle[enc] = curr_angle_map[enc] - zero_pos_map[enc];
//	angle[enc] = as5048a_normalize(angle[enc]);
//}

/**
 * @brief Get angle from encoder on selected motor.
 *
 */
void as5048a_getAngle(MotorDriver *driver){
	driver->curr_angle = as5048a_getRawRotation(driver->PIN_ENC);
	driver->curr_angle_map = as5048a_readToAngle(driver->curr_angle);

	driver->angle = driver->curr_angle_map - driver->zero_pos_map;
	driver->angle = as5048a_normalize(driver->angle);
}

/**
 * @brief Set zero angle from current angle
 */
void as5048a_setZero(MotorDriver *driver) {
	driver->zero_pos = as5048a_getRawRotation(driver->PIN_ENC);
	driver->zero_pos_map = as5048a_readToAngle(driver->zero_pos);
}

