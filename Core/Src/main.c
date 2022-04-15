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
#include "BMI270.h"
#include "AS5048A.h"
#include "Quaternions.h"
#include "DRV8313.h"
#include "def.h"
#include "foc.h"
#include "time_utils.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t rotary_cnt;
volatile uint8_t motor_control_toggle; //0 = control MotorX, 1 = control MotorY

MotorDriver MotorX;
MotorDriver MotorY;

IMU Imu;

float target_roll, target_pitch;

volatile uint16_t adc_read[5];
volatile uint8_t adcConvComplete = 0;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

char *buff[BUFF_SIZE];    //string buffer for UART-printing

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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

	/*Quaternion vars*/
	//######################################

	volatile clock_t us_t = 0;
	volatile clock_t us_t_prev = 0;


	//wait variable for bypassing zero-values when calculating quaternion
	uint8_t waitFilterUpdate = 0;
	//######################################
	/*IMU variables*/
	//######################################
	float gyr_range, acc_range;
	float gyr_limiter = 0.7;

	volatile clock_t t1, t2;
	float loop_time;

//	float x_target;
	//######################################
	/*DRV8313 vars*/
	//######################################
	float current_sense_gain = CURRENT_SENSE_GAIN;    //INA2181A1 gain = 20V/V
	float adc_ref = ADC_REF_VOLTAGE;    //reference voltage for STM32 ADC
	volatile float ina_ref = 0.0f;    //should be at 1.65 V for INA2181
	uint16_t adc_max = 4096;    //maximum ADC digital output (12-bit --> 4096)
	float adc_ratio;    //divisor by adc_mac
	float adc_factor;
	float sense_resistance = CURRENT_SENSE_RESISTANCE;    //3 parallel 0.68 Ω resistors as low-side sensors
	float sense_ratio;    //divisor by current_sense_gain

	//######################################

	uint32_t loop_cnt = 0;
	uint32_t print_flag = 700;    //how many loops should run before printing values

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	if (!USE_IMU_VIS) {
		sprintf((char*) buff, "\r\n###########################\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) buff, strlen((char*) buff),
		HAL_MAX_DELAY);
	}

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	TIM3->CNT = 0.5f * htim3.Init.Period;


	DWT_Init();

	EulerAngles Euler;

//	MotorZ.PIN_ENC = PIN_ENC_Z;
//	MotorZ.PIN_nFAULT = PIN_nFAULT_Z;

	if (USE_BMI270) {
		bmi270_spi_init();

		bmi270_spi_init_check();    //if (!USE_IMU_VIS)

		bmi270_pwr_conf(BMI270_PWR_MODE_PERF);
		bmi270_spi_write_8(REG_GYR_RANGE, range_2000);
		bmi270_spi_write_8(REG_ACC_RANGE, range_16g);

		bmi270_getGyroRange(&Imu);
		bmi270_getAccelRange(&Imu);
		bmi270_calibrateInit(&Imu, 0);

		Imu.gyr_odr = gyr_odr_3k2; /* Set gyro 3dB LP-filter cutoff to 50Hz */
		Imu.gyr_bwp = gyr_norm;
		Imu.gyr_noise_perf = gyr_ulp;
		Imu.gyr_filter_perf = gyr_ulp;
		bmi270_setGyroConf(&Imu);
		bmi270_getGyroConf(&Imu);

		Imu.acc_odr = acc_odr_1k6; /* Set accel 3dB LP-filter cutoff to 50Hz */
		Imu.acc_bwp = acc_osr4_avg1;
		Imu.acc_filter_perf = acc_ulp;
		bmi270_setAccConf(&Imu);
		bmi270_getAccConf(&Imu);

		/* PID and filtering */
		Imu.roll_sp = 0;
		Imu.pitch_sp = 0;

//		Imu.imu_filter.Tf = 0.145f;
		Imu.update_ctr = 0;
		Imu.update_goal = 7;
	}

//	MotorDriver MotorZ;

	MotorX.PIN_ENC = PIN_ENC_X;
	MotorX.PIN_nFAULT = PIN_nFAULT_X;
	MotorY.PIN_ENC = PIN_ENC_Y;
	MotorY.PIN_nFAULT = PIN_nFAULT_Y;

	/* Initialize encoders for each motor
	 * Note: Do this before initializing the IMU.
	 * Or else, */
	if (USE_AS5048A) {
		as5048a_init(&MotorX);
		as5048a_init(&MotorY);
//		as5048a_init(&MotorZ);
	}

	if (USE_DRV8313) {
		FastTrigonometry_buildTable();

		/*#######################################*/
		/*############### MotorX ################*/
		/*#######################################*/
		drv8313_init(&MotorX, &htim1);

		/*Filtering*/
		MotorX.LPF_current_d.Tf = 0.1f;
		MotorX.LPF_current_q.Tf = MotorX.LPF_current_q.Tf;
		MotorX.LPF_velocity.Tf = 0.3f;
		MotorX.LPF_angle.Tf = 0.4f;
		MotorX.LPF_imu.Tf = 0.6f;

		/* d-regulator */
		MotorX.d_reg.Kp = 1.25f;
		MotorX.d_reg.Ki = 0.001f;
		MotorX.d_reg.Kd = 0.00001f;
		/* q-regulator */
		MotorX.q_reg.Kp = MotorX.d_reg.Kp;
		MotorX.q_reg.Ki = MotorX.d_reg.Ki;
		MotorX.q_reg.Kd = MotorX.d_reg.Kd;

		/* speed regulator */
		MotorX.speed_reg.Kp = 0.5f;
		MotorX.speed_reg.Ki = 0.00f;
		MotorX.speed_reg.Kd = 0.00f;

		/* position regulator */
		MotorX.pos_reg.Kp = 2.0f;
		MotorX.pos_reg.Ki = 0.0f;
		MotorX.pos_reg.Kd = 0.00f;

		/* imu regulator */
		MotorX.imu_reg.Kp = 7.0f;
		MotorX.imu_reg.Ki = 9.066f;
		MotorX.imu_reg.Kd = 0.0f;

		/*#######################################*/
		/*############### MotorY ################*/
		/*#######################################*/
		drv8313_init(&MotorY, &htim2);

		/*Filtering*/
		MotorY.LPF_current_d.Tf = 0.1f;
		MotorY.LPF_current_q.Tf = MotorY.LPF_current_d.Tf;
		MotorY.LPF_velocity.Tf = 0.3f;
		MotorY.LPF_angle.Tf = 0.4f;
		MotorY.LPF_imu.Tf = 0.6f;

		/* d-regulator */
		MotorY.d_reg.Kp = 1.25f;
		MotorY.d_reg.Ki = 0.001f;
		MotorY.d_reg.Kd = 0.00001f;
		/* q-regulator */
		MotorY.q_reg.Kp = MotorY.d_reg.Kp;
		MotorY.q_reg.Ki = MotorY.d_reg.Ki;
		MotorY.q_reg.Kd = MotorY.d_reg.Kd;

		/* speed regulator */
		MotorY.speed_reg.Kp = 0.5f;
		MotorY.speed_reg.Ki = 0.00f;
		MotorY.speed_reg.Kd = 0.00f;

		/* position regulator */
		MotorY.pos_reg.Kp = 2.0f;
		MotorY.pos_reg.Ki = 0.0f;
		MotorY.pos_reg.Kd = 0.0f;

		/* imu regulator */
		MotorY.imu_reg.Kp = 7.0f;
		MotorY.imu_reg.Ki = 10.0f;
		MotorY.imu_reg.Kd = 0.0f;

		MotorX.update_goal = 4;
		MotorY.update_goal = 4;

//		drv8313_setPID(&MotorX, REGULATE_IMU, 90, 0, 0);
//		drv8313_setPID(&MotorY);

//		drv8313_init(&MotorZ, &htim3);

		MotorX.pole_pairs = 11;
		MotorY.pole_pairs = 11;
//		MotorZ.pole_pairs = 11;

//		MotorX.offset = 5.0f*DEG_TO_RAD;
//		MotorY.offset = 5.0f*DEG_TO_RAD;

		/*Initialize ADC */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_read, 5);

		adc_ratio = 1 / (float) adc_max;
		sense_ratio = 1 / (float) (sense_resistance * current_sense_gain);
		adc_factor = adc_ratio * adc_ref;

		/* ADC interrupt wait */
		while (adcConvComplete == 0) {
		}
		adcConvComplete = 0;

		/* get reference voltage on INA2181 */
		ina_ref = adc_read[4] * adc_factor;

		/* get phase currents on each motor */
		MotorX.i_a = (adc_read[0] * adc_factor - ina_ref) * sense_ratio;
		MotorX.i_b = (adc_read[1] * adc_factor - ina_ref) * sense_ratio;

		MotorY.i_a = (adc_read[2] * adc_factor - ina_ref) * sense_ratio;
		MotorY.i_b = (adc_read[3] * adc_factor - ina_ref) * sense_ratio;
//
//		MotorZ.i_a = (adc_read[4] * adc_ratio * adc_ref - ina_ref)
//				* sense_ratio;
//		MotorZ.i_b = (adc_read[5] * adc_ratio * adc_ref - ina_ref)
//				* sense_ratio;
	}

//	TIM5->PSC = 84;
//	TIM5->ARR = 1000000 - 1;
//
//	HAL_TIM_Base_Start(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1) {

		t1 = HAL_GetTick();

		Imu.gyr_x = (int16_t) bmi270_read_gyro(AXIS_X) * Imu.inv_gyr_range;
		Imu.gyr_y = (int16_t) bmi270_read_gyro(AXIS_Y) * Imu.inv_gyr_range;
		Imu.gyr_z = (int16_t) bmi270_read_gyro(AXIS_Z) * Imu.inv_gyr_range;


		Imu.acc_x = (int16_t) bmi270_read_accel(AXIS_X) * Imu.inv_acc_range;
		Imu.acc_y = (int16_t) bmi270_read_accel(AXIS_Y) * Imu.inv_acc_range;
		Imu.acc_z = (int16_t) bmi270_read_accel(AXIS_Z) * Imu.inv_acc_range;

//			HAL_GPIO_WritePin(GPIOC, FLAG_WHILE_LOOP_DONE_Pin, SET);

		filterUpdate(Imu.gyr_x * DEG_TO_RAD, Imu.gyr_y * DEG_TO_RAD,
				Imu.gyr_z * DEG_TO_RAD, Imu.acc_x, Imu.acc_y, Imu.acc_z,
				loop_time * 1e-3);

		Euler = ToEulerAngles(q0, q1, q2, q3);
		Imu.roll = lpf_exec(&MotorX.LPF_imu, Euler.x);
		Imu.pitch = lpf_exec(&MotorY.LPF_imu, Euler.y);


		/* ADC DMA wait */
		while (adcConvComplete == 0) {
		}
		adcConvComplete = 0;

		if(!Imu.update_ctr){
			/* get reference voltage on INA2181 */
			ina_ref = adc_read[4] * adc_factor;

			/* get phase currents on the motor */
			MotorX.i_a = (adc_read[0] * adc_factor - ina_ref) * sense_ratio;
			MotorX.i_b = (adc_read[1] * adc_factor - ina_ref) * sense_ratio;

			MotorY.i_a = (adc_read[2] * adc_factor - ina_ref) * sense_ratio;
			MotorY.i_b = (adc_read[3] * adc_factor - ina_ref) * sense_ratio;

		}
		Imu.update_ctr = (Imu.update_ctr + 1) % Imu.update_goal;

		/* Control camera with IMU */
		if(USE_IMU_CONTROL){
			/* set SVPWM on the motor*/
			/*use target_roll for IMU control*/
			target_roll = PID_Update(&MotorX.imu_reg, Imu.roll_sp, Imu.roll);
			foc_update(&MotorX, target_roll);


			/*use target_pitch for IMU control*/
			target_pitch = PID_Update(&MotorY.imu_reg, Imu.pitch_sp, Imu.pitch);
			foc_update(&MotorY, target_pitch);
		}
		/* Control camera with joystick */
		else{
			if(TIM3->CNT >= htim3.Init.Period - 5){
				TIM3->CNT = htim3.Init.Period - 5;
			}
			else if(TIM3->CNT <= 5){
				TIM3->CNT = 5;
			}

			if(motor_control_toggle == AXIS_X){
				Imu.roll_sp = (0.5f * htim3.Init.Period - TIM3->CNT) * DEG_TO_RAD;
			}
			else if(motor_control_toggle == AXIS_Y){
				Imu.pitch_sp = (0.5f * htim3.Init.Period - TIM3->CNT) * DEG_TO_RAD;
			}
			/* set SVPWM on the motor*/
			/*use target_roll for IMU control*/
			foc_update(&MotorX, Imu.roll_sp);


			/*use target_pitch for IMU control*/
			target_pitch = PID_Update(&MotorY.imu_reg, Imu.pitch_sp, Imu.pitch);
			foc_update(&MotorY, Imu.pitch_sp);
		}

		if (USE_PRINT) {
			uint16_t len = 0;

			if (!USE_IMU_VIS && !loop_cnt) {
				if (USE_AS5048A) {
					len +=
							sprintf((char*) buff + len,
									"\r\n########## ENCODER ANGLES ##########\r\n"
											"MotorX\r\n"
											"Angle: %.3f\t Zero pos: %.3f\t speed: %.3frad/s\r\n"
											"MotorY\r\n"
											"Angle: %.3f\t Zero pos: %.3f\t speed: %.3frad/s\r\n",
									MotorX.angle * RAD_TO_DEG,
									MotorX.zero_pos_map * RAD_TO_DEG,
									MotorX.velocity, MotorY.angle * RAD_TO_DEG,
									MotorY.zero_pos_map * RAD_TO_DEG,
									MotorY.velocity);
				}
				if (USE_DRV8313) {
					len += sprintf((char*) buff + len,
							"\r\n########## DRV8313 DATA ##########\r\n"
									"######################################\r\n"
									"MotorX\r\n"
									"i_a: %.3f\ti_b: %.3f\r\n"
									"MotorY\r\n"
									"i_a: %f\ti_b: %f\r\n"
//							"MotorZ\r\n"
//							"i_a: %f\ti_b: %f\r\n"
									"INA181 ref. voltage: %.3f\r\n", MotorX.i_a,
							MotorX.i_b, MotorY.i_a, MotorY.i_b,
//							MotorZ.i_a, MotorZ.i_b,
							ina_ref);
				}
				if (USE_BMI270) {
					len +=
							sprintf((char*) buff + len,
									"\r\n########## IMU DATA ##########\r\n"
											"gyr_range: %f\r\n"
											"acc_range: %f\r\n"
											"loop time (ms): %f\r\n"
											"gyroscope x: %f˚/s, y: %f˚/s, z: %f˚/s\r\n"
											"accelerometer x: %f m/s2, y: %f m/s2, z: %f m/s2\r\n"
											"q0: %f, q1: %f, q2: %f, q3: %f\r\n"
											"roll: %f, pitch: %f, yaw: %f\r\n"
											"roll_sp: %f, pitch_sp: %f, yaw_sp: %f\r\n",
									Imu.acc_range, Imu.gyr_range, loop_time,
									Imu.gyr_x, Imu.gyr_y, Imu.gyr_z, Imu.acc_x,
									Imu.acc_y, Imu.acc_z, q0, q1, q2, q3,
									Imu.roll * RAD_TO_DEG,
									Imu.pitch * RAD_TO_DEG,
									Imu.yaw * RAD_TO_DEG,
									Imu.roll_sp * RAD_TO_DEG,
									Imu.pitch_sp * RAD_TO_DEG,
									Imu.yaw_sp * RAD_TO_DEG);
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) buff,
						strlen((char*) buff),
						HAL_MAX_DELAY);
			}
			if (USE_IMU_VIS) {
				len += sprintf((char*) buff + len,
						"w%.4fwa%.4fab%.4fbc%.4fcy%.4fyp%.4fpr%.4fr\r\n", q0,
						q1, q2, q3, Imu.yaw, Imu.pitch, Imu.roll);
				HAL_UART_Transmit(&huart2, (uint8_t*) buff,
						strlen((char*) buff),
						HAL_MAX_DELAY);
			}

		}

		loop_cnt++;
		loop_cnt %= print_flag;

		/* millisecond timer */
		/* Note: It seems that HAL_GetTick() works for quaternions, while __HAL_TIM_GET_COUNTER() does not.
		 * The latter results in jittery movement. */
		t2 = HAL_GetTick() - t1;
		loop_time = t2;

		HAL_GPIO_WritePin(GPIOC, FLAG_WHILE_LOOP_DONE_Pin, RESET);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1024-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1024-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 730;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(FLAG_FUNCTION_DONE_GPIO_Port, FLAG_FUNCTION_DONE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLAG_WHILE_LOOP_DONE_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BMI270_CS_Pin|MotorX_encoder_CS_Pin|MotorY_encoder_CS_Pin|MotorZ_encoder_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reset_program_Pin */
  GPIO_InitStruct.Pin = Reset_program_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Reset_program_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAG_FUNCTION_DONE_Pin */
  GPIO_InitStruct.Pin = FLAG_FUNCTION_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLAG_FUNCTION_DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAG_WHILE_LOOP_DONE_Pin */
  GPIO_InitStruct.Pin = FLAG_WHILE_LOOP_DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLAG_WHILE_LOOP_DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI270_CS_Pin MotorX_encoder_CS_Pin MotorY_encoder_CS_Pin MotorZ_encoder_CS_Pin */
  GPIO_InitStruct.Pin = BMI270_CS_Pin|MotorX_encoder_CS_Pin|MotorY_encoder_CS_Pin|MotorZ_encoder_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : nFAULT_Z_Pin nFAULT_X_Pin nFAULT_Y_Pin */
  GPIO_InitStruct.Pin = nFAULT_Z_Pin|nFAULT_X_Pin|nFAULT_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nSLEEP_Pin */
  GPIO_InitStruct.Pin = nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nSLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_selector_Pin */
  GPIO_InitStruct.Pin = Motor_selector_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Motor_selector_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcConvComplete = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12){
		motor_control_toggle = !motor_control_toggle;
	}
}

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

