/*
 * DRV8313.h
 *
 *  Created on: Nov 17, 2021
 *      Author: maxborglowe
 */

#ifndef INC_DRV8313_H_
#define INC_DRV8313_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "def.h"

extern ADC_HandleTypeDef hadc1;

#define PINBUS_DRV8313 GPIOB
#define PIN_nSLEEP GPIO_PIN_13
#define PIN_nFAULT GPIO_PIN_14


struct Drv8313_Driver{
	uint16_t sense1, sense2, sense3; //current sensors
	uint8_t motor_id;

};

uint8_t drv8313_init(void);
void drv8313_sense(struct Drv8313_Driver driver);

#endif /* INC_DRV8313_H_ */
