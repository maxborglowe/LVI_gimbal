/*
 * time_utils.h
 *
 *  Created on: Jan 6, 2022
 *      Author: maxborglowe
 */

#ifndef INC_TIME_UTILS_H_
#define INC_TIME_UTILS_H_

#include "stm32f4xx_hal.h"

uint32_t get_us();

__STATIC_INLINE void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

__STATIC_INLINE void delay_us(uint32_t us)
{
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

__STATIC_INLINE uint32_t micros(void){
	return  DWT->CYCCNT / (SystemCoreClock * 1e-6);
}

#endif /* INC_TIME_UTILS_H_ */
