/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Reset_program_Pin GPIO_PIN_13
#define Reset_program_GPIO_Port GPIOC
#define MotorY_i_b_Pin GPIO_PIN_0
#define MotorY_i_b_GPIO_Port GPIOC
#define MotorZ_i_a_Pin GPIO_PIN_1
#define MotorZ_i_a_GPIO_Port GPIOC
#define MotorZ_i_b_Pin GPIO_PIN_2
#define MotorZ_i_b_GPIO_Port GPIOC
#define current_sense_REF_Pin GPIO_PIN_3
#define current_sense_REF_GPIO_Port GPIOC
#define MotorX_i_a_Pin GPIO_PIN_0
#define MotorX_i_a_GPIO_Port GPIOA
#define MotorY_pwm2_Pin GPIO_PIN_1
#define MotorY_pwm2_GPIO_Port GPIOA
#define MotorX_i_b_Pin GPIO_PIN_4
#define MotorX_i_b_GPIO_Port GPIOA
#define FLAG_WHILE_LOOP_DONE_Pin GPIO_PIN_4
#define FLAG_WHILE_LOOP_DONE_GPIO_Port GPIOC
#define BMI270_CS_Pin GPIO_PIN_0
#define BMI270_CS_GPIO_Port GPIOB
#define MotorY_i_a_Pin GPIO_PIN_1
#define MotorY_i_a_GPIO_Port GPIOB
#define MotorY_pwm3_Pin GPIO_PIN_10
#define MotorY_pwm3_GPIO_Port GPIOB
#define nFAULT_Z_Pin GPIO_PIN_12
#define nFAULT_Z_GPIO_Port GPIOB
#define nSLEEP_Pin GPIO_PIN_13
#define nSLEEP_GPIO_Port GPIOB
#define nFAULT_X_Pin GPIO_PIN_14
#define nFAULT_X_GPIO_Port GPIOB
#define nFAULT_Y_Pin GPIO_PIN_15
#define nFAULT_Y_GPIO_Port GPIOB
#define MotorZ_pwm1_Pin GPIO_PIN_6
#define MotorZ_pwm1_GPIO_Port GPIOC
#define MotorZ_pwm2_Pin GPIO_PIN_7
#define MotorZ_pwm2_GPIO_Port GPIOC
#define MotorZ_pwm3_Pin GPIO_PIN_8
#define MotorZ_pwm3_GPIO_Port GPIOC
#define MotorX_pwm1_Pin GPIO_PIN_8
#define MotorX_pwm1_GPIO_Port GPIOA
#define MotorX_pwm2_Pin GPIO_PIN_9
#define MotorX_pwm2_GPIO_Port GPIOA
#define MotorX_pwm3_Pin GPIO_PIN_10
#define MotorX_pwm3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MotorY_pwm1_Pin GPIO_PIN_15
#define MotorY_pwm1_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MotorX_encoder_CS_Pin GPIO_PIN_4
#define MotorX_encoder_CS_GPIO_Port GPIOB
#define MotorY_encoder_CS_Pin GPIO_PIN_5
#define MotorY_encoder_CS_GPIO_Port GPIOB
#define MotorZ_encoder_CS_Pin GPIO_PIN_6
#define MotorZ_encoder_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
