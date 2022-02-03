/*
 * time_utils.c
 *
 *  Created on: Jan 6, 2022
 *      Author: maxborglowe
 */

#include "time_utils.h"

uint32_t get_us(){
	return HAL_GetTick()*1000;
}

