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
#include <string.h>
#include <stdio.h>
#include "ICM20602.h"
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

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef status;

uint8_t buff[64]; //string buff for UART
uint16_t gyro_x_data, gyro_y_data, gyro_z_data; //raw gyro data



//SA0 = 0 --> Arbitrary address value. If two gyros are used,
//also an address ending with SA0 = 1.
static const uint8_t ICM20602_ADDR = 0b1101000<<1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef icm20602_write_config(uint8_t config);
HAL_StatusTypeDef icm20602_write_i2c_if(uint8_t config);
HAL_StatusTypeDef icm20602_write_user_ctrl(uint8_t config);
HAL_StatusTypeDef icm20602_write_smplrt_div(uint8_t config);

HAL_StatusTypeDef icm20602_read_gyro_offset(uint16_t *offset, uint8_t axis);
HAL_StatusTypeDef icm20602_read_gyro(uint16_t *deg, uint8_t axis);
HAL_StatusTypeDef icm20602_read_pwr_1(void);
HAL_StatusTypeDef icm20602_write_pwr_1(uint8_t config);
HAL_StatusTypeDef icm20602_read_pwr_2(void);
HAL_StatusTypeDef icm20602_write_pwr_2(uint8_t config);
HAL_StatusTypeDef icm20602_read_gyro_config(void);
HAL_StatusTypeDef icm20602_write_gyro_config(uint8_t config);
HAL_StatusTypeDef icm20602_read_whoami(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //reset the ICM20602

	icm20602_write_pwr_1(0b10000000);
	HAL_Delay(1000); //reset delay
	icm20602_read_whoami();			//verify chip --> output 0x12
	icm20602_write_pwr_1(0b00000001);	//set clock to internal PLL
	icm20602_write_pwr_2(0b00111111);		//place accel and gyro in standby
	icm20602_write_smplrt_div(0x07);
	icm20602_write_user_ctrl(0x00);	//disable fifo
	icm20602_write_i2c_if(0x00); 	//enable i2c
	icm20602_write_config(0b00000001);
	icm20602_write_gyro_config((0b00011000) | 0x02);

	icm20602_write_pwr_2(0b00111000); //enable gyro disable accel
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint16_t rotation_x, rotation_y, rotation_z;
	  uint16_t offset_x, offset_y, offset_z;

	  icm20602_read_pwr_1();
	  icm20602_read_pwr_2();

	  icm20602_read_gyro_config(); //set ±2000 dps, no self test
	  icm20602_read_gyro(&rotation_x, ICM20602_GYRO_AXIS_X);
	  icm20602_read_gyro(&rotation_y, ICM20602_GYRO_AXIS_Y);
	  icm20602_read_gyro(&rotation_z, ICM20602_GYRO_AXIS_Z);

	  rotation_x = (float)rotation_x/16.4;
	  rotation_y = (float)rotation_y/16.4;
	  rotation_z = (float)rotation_z/16.4;

	  sprintf((char*)buff,
			  "gyro x: %u˚, y: %u˚, z: %u˚ \r\n",
			  rotation_x, rotation_y, rotation_z
			  );

	  //Output UART buffer (gyro data)
	  HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);

	  icm20602_read_gyro_offset(&offset_x, ICM20602_GYRO_AXIS_X);
	  icm20602_read_gyro_offset(&offset_y, ICM20602_GYRO_AXIS_Y);
	  icm20602_read_gyro_offset(&offset_z, ICM20602_GYRO_AXIS_Z);

	  sprintf((char*)buff,
	  			  "gyro x: %u, y: %u, z: %u \r\n",
	  			  offset_x, offset_y, offset_z
	  			  );

	  	  //Output UART buffer (gyro data)
	  	  HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);
	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

HAL_StatusTypeDef icm20602_write(uint8_t config, uint8_t reg){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(reg), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write");
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_config(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_CONFIG), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write REG_I2C_IF\r\n");
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_i2c_if(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_I2C_IF), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write REG_I2C_IF\r\n");
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_smplrt_div(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_SMPLRT_DIV), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write REG_I2C_IF\r\n");
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_user_ctrl(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_USER_CTRL), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write REG_I2C_IF\r\n");
		return status;
	}
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_pwr_1(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_PWR_MGMT_1), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write PWR_MGMT_1\r\n");
		return status;
	}
	return HAL_OK;
}


HAL_StatusTypeDef icm20602_read_pwr_1(void){
	uint8_t data;

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_PWR_MGMT_1), 0x01, &data, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read PWR_MGMT_1\r\n");
		return status;
	}

	sprintf((char*)buff,
				  "power config: %u \r\n",
				  data
				  );

	//Output UART buffer (gyro data)
	HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);

	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_pwr_2(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_PWR_MGMT_2), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write PWR_MGMT_2\r\n");
		return status;
	}
	return HAL_OK;
}


HAL_StatusTypeDef icm20602_read_pwr_2(void){
	uint8_t data;

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_PWR_MGMT_2), 0x01, &data, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read PWR_MGMT_2\r\n");
		return status;
	}

	sprintf((char*)buff,
				  "power 2 config: %u \r\n",
				  data
				  );

	//Output UART buffer (gyro data)
	HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);

	return HAL_OK;
}


/**
 * @param variable onto which angular value is declared
 * @param which axis to be measured --> 0 = x, 1 = y, 2 = z
 */
HAL_StatusTypeDef icm20602_read_gyro(uint16_t *deg, uint8_t axis){
	uint8_t data[2];

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_GYRO_XOUT_H + 2*axis), 0x01, &(data[0]), 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read GYRO_XOUT_H\r\n");
		return status;
	}
	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_GYRO_XOUT_L + 2*axis), 0x01, &(data[1]), 1, HAL_MAX_DELAY);
		if(status != HAL_OK){
			strcpy((char*)buff, "Error read GYRO_XOUT_L\r\n");
			return status;
		}

	*deg = (data[0]<<8 | data[1]);
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_read_gyro_offset(uint16_t *offset, uint8_t axis){
	uint8_t data[2];

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_XG_OFFS_USRH + 2*axis), 0x01, &(data[0]), 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read GYRO_XOUT_H\r\n");
		return status;
	}
	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_XG_OFFS_USRL + 2*axis), 0x01, &(data[1]), 1, HAL_MAX_DELAY);
		if(status != HAL_OK){
			strcpy((char*)buff, "Error read GYRO_XOUT_L\r\n");
			return status;
		}

	*offset = (data[0]<<8 | data[1]);
	return HAL_OK;
}


HAL_StatusTypeDef icm20602_read_whoami(void){
	uint8_t data;

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_WHO_AM_I), 0x01, &data, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read PWR_MGMT_1\r\n");
		return status;
	}
	sprintf((char*)buff,
				  "who am i response: %u \r\n",
				  data
				  );
	HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_read_gyro_config(void){
	uint8_t data;

	status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ICM20602_ADDR | 0x01), (uint16_t)(REG_GYRO_CONFIG), 0x01, &data, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error read PWR_MGMT_1\r\n");
		return status;
	}
	sprintf((char*)buff,
				  "gyro config: %u \r\n",
				  data
				  );
	HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);
	return HAL_OK;
}

HAL_StatusTypeDef icm20602_write_gyro_config(uint8_t config){

	status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(ICM20602_ADDR), (uint16_t)(REG_GYRO_CONFIG), 0x01, &config, 1, HAL_MAX_DELAY);
	if(status != HAL_OK){
		strcpy((char*)buff, "Error write GYRO_CONFIG\r\n");
		return status;
	}

	return HAL_OK;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
