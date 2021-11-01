/*
 * ICM20602.c
 *
 *  Created on: Oct 29, 2021
 *      Author: maxborglowe
 */

#include "ICM20602.h"


float Get_FS_SEL(uint8_t param){
	uint8_t read_conf;
	float out;

	i2c_read_8(ICM20602_ADDR, REG_GYRO_CONFIG, &read_conf);

	read_conf = read_conf >> 3;

	if(param == 0){
		switch(read_conf){
		//±250 dps
		case 0:
			out = 131;
			break;

		//±500 dps
		case 1:
			out = 65.5;
			break;

		//±1000 dps
		case 2:
			out = 32.8;
			break;

		//±2000 dps
		case 3:
			out = 16.4;
			break;
		}
	} else if (param == 1){
		switch(read_conf){
		//±250 dps
		case 0:
			out = 250;
			break;

		//±500 dps
		case 1:
			out = 500;
			break;

		//±1000 dps
		case 2:
			out = 1000;
			break;

		//±2000 dps
		case 3:
			out = 2000;
			break;
		}
	}

	return out;
}

/*
 * Returns a value based on the AFS_SEL configuration.
 * This is used to divide the raw gyro data.
 */
float Get_AFS_SEL(void){
	uint8_t read_conf;
	uint16_t out;

	i2c_read_8(ICM20602_ADDR, REG_ACCEL_CONFIG, &read_conf);

	read_conf = read_conf >> 3;

	switch(read_conf){
	//±2 g
	case 0:
		out = 16384.0f;
		break;

	//±4 g
	case 1:
		out = 8192.0f;
		break;

	//±8 g
	case 2:
		out = 4096.0f;
		break;

	//±16 g
	case 3:
		out = 2048.0f;
		break;
	}
	return out;
}

float Get_SampleRate(void){
	uint8_t read_var;
	i2c_read_8(ICM20602_ADDR, REG_GYRO_CONFIG, &read_var);
	FCHOICE_B = read_var & (0b011);
	i2c_read_8(ICM20602_ADDR, REG_CONFIG, &read_var);
	DLPF_CFG = read_var & (0b111);
	i2c_read_8(ICM20602_ADDR, REG_SMPLRT_DIV, &read_var);
	SMPLRT_DIV = read_var;

	if(FCHOICE_B > 0 && SMPLRT_DIV == 0){
		return 32000;
	}
	else if(FCHOICE_B == 0 && SMPLRT_DIV == 0){
		if(DLPF_CFG == 0 || DLPF_CFG == 7){
			return 8000;
		}
	}
	return 1000;
}

