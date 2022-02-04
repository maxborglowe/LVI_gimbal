/*
 * DRV8313.c
 *
 *  Created on: Nov 17, 2021
 *      Author: maxborglowe
 */

#include "DRV8313.h"




/**
 * @brief Initialize the DRV8313
 */
uint8_t drv8313_init(){
	HAL_GPIO_WritePin(PINBUS_DRV8313, PIN_nSLEEP, GPIO_PIN_SET); //Enable the unit by setting nRESET + nSLEEP to HIGH
	if(!HAL_GPIO_ReadPin(PINBUS_DRV8313, PIN_nFAULT)){
		return 0;
	}
	HAL_GPIO_WritePin(PINBUS_DRV8313, PIN_nSLEEP, GPIO_PIN_SET);
	return 1;

}

//
//void drv8313_sense(struct Drv8313_Driver driver){
//
//	driver.sense1 = adc_read[0+driver.motor_id];
//	driver.sense2 =	adc_read[1+driver.motor_id];
//	driver.sense3 =	adc_read[2+driver.motor_id];
//}




