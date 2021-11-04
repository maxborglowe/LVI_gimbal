/*
 * spi.c
 *
 *  Created on: Nov 3, 2021
 *      Author: maxborglowe
 */

#include <AS5048A.h>
#include <math.h>

extern uint16_t zero_pos;

/*
 * @brief Read 16-bit data at selected register.
 * @param SPI slave select pin.
 * @param AS5048A register
 */
uint16_t as5048a_read(uint16_t ss, uint16_t reg){
	uint8_t data[2];

	uint16_t cmd = CMD_READ | reg;
	cmd |= ((uint16_t)calcEvenParity(cmd)<<15);

	data[1] = cmd & 0xFF;
	data[0] = (cmd>>8) & 0xFF;

	HAL_GPIO_WritePin(GPIOB, ss, 0);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(GPIOB, ss, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, ss, 0);
	HAL_SPI_Receive(&hspi1, (uint8_t*)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(GPIOB, ss, 1);

	return (((data[0] & 0xFF)<<8) | (data[1] & 0xFF)) & ~0xC000;
}

/*
 * @brief Get raw rotation as 16-bit value.
 * @param SPI slave select pin.
 */
uint16_t as5048a_getRawRotation(uint16_t ss){
	return as5048a_read(ss, REG_ANGLE);
}

//int as5048a_getRotation(uint16_t ss){
//	uint16_t data;
//	int rot;
//
//	data = as5048a_read(ss, REG_ANGLE);
//	rot = (int)data - (int)bldc_pos;
//	if(rot > 8191){
//		rot = -((REG_ANGLE) - rot);
//	}
//	return rot;
//}

/*
 * @brief Normalized the input angle, meaning that the angle starts at 0˙ and ends at 360˙
 * @param unnormalized input angle
 */
float as5048a_normalize(float angle){
	//angle += 180;
	angle = fmod(angle, 360);
	if (angle < 0){
		angle += 360;
	}
	//angle -= 180;
	return angle;
}

/*
 * @brief Set the zero position to arbitrary value.
 * Useful: When moving the camera by hand, you could make the zero position the same as the current angle
 * @param input angle
 */
void as5048a_setZero(uint16_t arg_pos){
	zero_pos = arg_pos % 360;
}

float as5048a_readToAngle(uint16_t angle) {
	return 2*((float)angle * ((float)360 / 16383) - 180);
};

uint8_t calcEvenParity(uint16_t value){
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++){
		if (value & 0x1){
			cnt++;
		}

		value >>= 1;
	}
	return cnt & 0x1;
}
