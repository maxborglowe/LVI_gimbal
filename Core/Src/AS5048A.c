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

	//HAL_Delay(1);

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

	as5048a_setZero(driver);

	as5048a_getAngle(driver);

	HAL_Delay(1);

	lpf_init(&driver->LPF_angle_measure, 0.005);
}

/*
 * @brief Normalized the input angle, meaning that the angle starts at 0˚ and ends at 360˚
 * @param unnormalized input angle
 */
float as5048a_normalize(float angle) {
	angle += 180;
	angle = fmod(angle, 360);
	if (angle < 0) {
		angle += 360;
	}
	angle -= 180;
	return angle;
}

/*
 * @brief Normalized the input angle, meaning that the angle starts at 0 and ends at 2PI
 * @param unnormalized input angle
 */
float as5048a_normalizeRad(float angle) {
	angle += _PI;
	angle = fmod(angle, _2PI);
	if (angle < 0) angle += _2PI;
	angle -= _PI;
	return angle;
}


/*
 * @brief Set the zero position to arbitrary value.
 * Use: When moving the camera by hand, you could make the zero position the same as the current angle
 * @param input angle
 */
void as5048a_setZeroArg(MotorDriver *driver, float arg_pos) {
	driver->zero_pos = fmod(arg_pos, 360);
}

/*
 * @brief Convert raw data from getRawRotation to angles in degrees.
 * @param Raw angular data input.
 */
float as5048a_readToAngle(uint16_t angle) {
	return 2 * ((float) angle * (360.0f * _1_16384) - 180);
}

/*
 * @brief Convert raw data from getRawRotation to angles in degrees.
 * @param Raw angular data input.
 */
float as5048a_readToAngleRad(uint16_t angle) {
	return 2 * ((float) angle * (_2PI * _1_16384) - _PI);
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

/** @brief Get angle from encoder on selected motor.
 *
 */
void as5048a_getAngle(MotorDriver *driver) {
	float angle = as5048a_readToAngle(as5048a_getRawRotation(driver->PIN_ENC)); //- driver->zero_pos_map;
	driver->angle = lpf_exec(&driver->LPF_angle_measure, as5048a_normalize(angle));
}

/** @brief Get angle from encoder on selected motor.
 *
 */
void as5048a_getAngleRad(MotorDriver *driver) {
	float angle = as5048a_readToAngleRad(as5048a_getRawRotation(driver->PIN_ENC)); //- driver->zero_pos_map;
	driver->angle = lpf_exec(&driver->LPF_angle_measure, as5048a_normalizeRad(angle));
}

/**
 * @brief Set zero angle from current angle
 */
void as5048a_setZero(MotorDriver *driver) {
	driver->zero_pos = as5048a_getRawRotation(driver->PIN_ENC);
	driver->zero_pos_map = as5048a_readToAngle(driver->zero_pos);
}

/**@brief Calculate speed in ˚/s using angular values and input time
 *@param Motor driver containing anglular values
 */
void as5048a_getVelocity(MotorDriver *driver){

//	as5048a_getAngle(driver);
	uint32_t timestamp_us = get_us();

	float T_samp = (timestamp_us - driver->prev_timestamp_us) * 1e-6f;
	if(T_samp <= 0 || T_samp > 0.5f) T_samp = 1e-3;

	/* Calculate difference between current and previous angles */
	float angle_diff = fabs(fmod(driver->angle - driver->prev_angle + 180, 360) - 180);

	/* Calculate velocity */
	driver->velocity = angle_diff/T_samp;
	if(T_samp == 0) driver->velocity = 0;

	driver->prev_timestamp_us = timestamp_us;
	driver->prev_angle = driver->angle;
}

