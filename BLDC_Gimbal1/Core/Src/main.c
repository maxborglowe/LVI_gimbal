/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "ICM20602.h"
#include "BMI270.h"
#include "I2C.h"
#include "AS5048A.h"
#include "Quaternions.h"
//#include "Madgwick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef status;

/*GPIO aliases*/
//######################################
//######################################
/*Quaternion vars*/
//######################################
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
struct EulerAngles euler;
float sampleDelay = 50.0f; //milliseconds

clock_t t1;
clock_t t2;
float while_t = 74.0f;

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795457
//######################################

/*IMU variables*/
//######################################
int16_t rot_x_raw, rot_y_raw, rot_z_raw;
int16_t acc_x_raw, acc_y_raw, acc_z_raw;
float rot_x, rot_y, rot_z;
float acc_x, acc_y, acc_z;

float roll, pitch, yaw;

uint16_t offset_x = 0, offset_y = 0, offset_z = 0;

uint8_t chip_id;
//######################################

/*Encoder vars*/
//######################################
uint16_t curr_angle[amt_encoders];
float curr_angle_map[amt_encoders];
uint16_t zero_pos[amt_encoders];
float zero_pos_map[amt_encoders];

float angle[amt_encoders];
//######################################

/*Simulation vars*/
//######################################
float ax_s = 0, ay_s = 0, az_s = 0, gx_s = 0, gy_s = 0, gz_s = 0;
uint8_t sim_ena = 1;
uint8_t zero_c = 0;
//######################################

//wait variable for bypassing zero-values when calculating quaternion
uint8_t waitUpdate = 0;

uint8_t buff[512]; //string buff for UART

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	icm20602_init();
	//i2c_read_8(BMI270_ADDR, REG_CHIP_ID, &chip_id);

	as5048a_init();

	//Set zero position for all encoders
	zero_pos[ENC_X] = as5048a_getRawRotation(GPIO_ENC_X);
	zero_pos[ENC_Y] = as5048a_getRawRotation(GPIO_ENC_Y);
	zero_pos[ENC_Z] = as5048a_getRawRotation(GPIO_ENC_Z);
	zero_pos_map[ENC_X] = as5048a_readToAngle(zero_pos[ENC_X]);
	zero_pos_map[ENC_Y] = as5048a_readToAngle(zero_pos[ENC_Y]);
	zero_pos_map[ENC_Z] = as5048a_readToAngle(zero_pos[ENC_Z]);

	setSampleFreq();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		t1 = HAL_GetTick();
		setSampleFreq();

		if (!sim_ena) {
			for (uint8_t c = 0; c < amt_encoders; c++) {
				curr_angle[c] = as5048a_getRawRotation(GPIO_ENC_X << c);
				curr_angle_map[c] = as5048a_readToAngle(curr_angle[c]);

				angle[c] = curr_angle_map[c] - zero_pos_map[c];
				angle[c] = as5048a_normalize(angle[c]);

				const char *enc = getEncoderName(c);

				sprintf((char*) buff, "zero pos %s: %f\r\n"
						"encoder: %f\r\n", enc, zero_pos_map[c], angle[c]);
				HAL_UART_Transmit(&huart2, buff, strlen((char*) buff),
						HAL_MAX_DELAY);
			}

			i2c_read_16(ICM20602_ADDR, REG_ACCEL_XOUT_L, REG_ACCEL_XOUT_H, &acc_x_raw);
			i2c_read_16(ICM20602_ADDR, REG_ACCEL_YOUT_L, REG_ACCEL_YOUT_H, &acc_y_raw);
			i2c_read_16(ICM20602_ADDR, REG_ACCEL_ZOUT_L, REG_ACCEL_ZOUT_H, &acc_z_raw);

			i2c_read_16(ICM20602_ADDR, REG_GYRO_XOUT_L, REG_GYRO_XOUT_H, &rot_x_raw);
			i2c_read_16(ICM20602_ADDR, REG_GYRO_YOUT_L, REG_GYRO_YOUT_H, &rot_y_raw);
			i2c_read_16(ICM20602_ADDR, REG_GYRO_ZOUT_L, REG_GYRO_ZOUT_H, &rot_z_raw);

			rot_x = rot_x_raw/FS_SEL_divider;
			rot_y = rot_y_raw/FS_SEL_divider;
			rot_z = rot_z_raw/FS_SEL_divider;

			acc_x = acc_x_raw/AFS_SEL;
			acc_y = acc_y_raw/AFS_SEL;
			acc_z = acc_z_raw/AFS_SEL;
		}



		//Wait before updating quaternion. This avoids div by zero in different Quaternion functions.
		if (waitUpdate > 1 && !sim_ena) {
			filterUpdate(rot_x, rot_y, rot_z, acc_x, acc_y, acc_z);
		} else if (sim_ena) {
			gx_s = 0;
			gy_s = 0;
			gz_s = 10;
			ax_s = 0.0;
			ay_s = 0.0;
			az_s = 0.0;
			filterUpdate(gx_s * DEG_TO_RAD, gy_s * DEG_TO_RAD,
					gz_s * DEG_TO_RAD, ax_s, ay_s, az_s);

//			zero_c %= 10;
//			if (zero_c == 0) {
//				ax_s = 0, ay_s = 0, az_s = 0, gx_s = 0, gy_s = 0, gz_s = 0;
//			}
//			zero_c++;
		}

		waitUpdate++;
		waitUpdate %= 20;

		euler = ToEulerAngles(q0, q1, q2, q3);

		roll = as5048a_normalize(euler.x * RAD_TO_DEG);
		pitch = euler.y * RAD_TO_DEG;
		yaw = as5048a_normalize(euler.z * RAD_TO_DEG);

		sprintf((char*) buff,
				"while loop time: %f\r\n"
				"gyroscope x: %f˚/s, y: %f˚/s, z: %f˚/s\r\n"
				"accelerometer x: %f m/s2, y: %f m/s2, z: %f m/s2\r\n"
				"q1: %f, q2: %f, q3: %f, q4: %f\r\n"
				"roll: %f, pitch: %f, yaw: %f\r\n",
//			  rot_x, rot_y, rot_z,
//			  acc_x, acc_y, acc_z,
				while_t, gx_s, gy_s, gz_s, ax_s, ay_s, az_s, q0, q1, q2, q3, roll, pitch,
				yaw);

		HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);

		HAL_Delay(sampleDelay);
		t2 = HAL_GetTick();
		while_t = t2-t1;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB5 PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
