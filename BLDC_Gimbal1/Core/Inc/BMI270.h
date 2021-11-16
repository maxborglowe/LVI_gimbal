/*
 * BMI270.h
 *
 *  Created on: Nov 1, 2021
 *      Author: maxborglowe
 */

#ifndef INC_BMI270_H_
#define INC_BMI270_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

//Default address. If this address is used, SDO must be pulled to GND.
#define BMI270_ADDR 0b1101000

#define CHIP_ID 0x24			//Expected response when reading REG_CHIP_ID

//BMI270 registers
#define REG_CHIP_ID 0x00
#define REG_DATA_8 0x0C //accelerometer x, bits 0 - 7
#define REG_DATA_9 0x0D	//accelerometer x, bits 8 - 15
#define REG_DATA_10 0x0E
#define REG_DATA_11 0x0F
#define REG_DATA_12 0x10 //accelerometer z, bits 0 - 7
#define REG_DATA_13 0x11 //accelerometer z, bits 8 - 15
#define REG_DATA_14 0x12 //gyroscope x, bits 0 - 7
#define REG_DATA_15 0x13 //gyroscope x, bits 8 - 15
#define REG_DATA_16 0x14
#define REG_DATA_17 0x15
#define REG_DATA_18 0x16 //gyroscope z, bits 0 - 7
#define REG_DATA_19 0x17 //gyroscope z, bits 8 - 15
#define REG_INTERNAL_STATUS 0x21
#define REG_GYR_CAS 0x3C
//########################
#define REG_ACC_CONF 0x40
	#define reserved 0x00
	#define odr_0p78 0x01
	#define odr_1p5 0x02
	#define odr_3p1 0x03
	#define odr_6p25 0x04
	#define odr_12p5 0x05
	#define odr_25 0x06
	#define odr_50 0x07
	#define odr_100 0x08
	#define odr_200 0x09
	#define odr_400 0x0A
	#define odr_800 0x0B
	#define odr_1k6 0x0C
	#define odr_3k2 0x0D
	#define odr_6k4 0x0E
	#define odr_12k8 0x0F
//########################
#define REG_ACC_RANGE 0x41
	#define range_2g 0x00
	#define range_4g 0x01
	#define range_8g 0x02
	#define range_16g 0x03
//########################
#define REG_GYR_CONF 0x42
//########################
#define REG_GYR_RANGE 0x43
#define range_2000 0x00
#define range_1000 0x01
#define range_500 0x02
#define range_250 0x03
#define range_125 0x04
//########################
#define REG_INIT_CTRL 0x59
#define REG_INIT_DATA 0x5E
#define REG_NV_CONF 0x70
#define REG_PWR_CONF 0x7C
#define REG_PWR_CTRL 0x7D
	#define aux_en 0x00<<0
	#define	gyr_en 0x01<<1
	#define acc_en 0x01<<2
	#define temp_en 0x01<<3

//customized BMI270 stuff
#define BMI270_CS GPIO_PIN_10
#define BMI270_SPI 1

#define BMI270_PWR_MODE_LOW 0
#define BMI270_PWR_MODE_NORM 1
#define BMI270_PWR_MODE_PERF 2

#define BMI270_AXIS_X 0
#define BMI270_AXIS_Y 1
#define BMI270_AXIS_Z 2

#define BMI270_GYRO_2000_DPS 16.4f
#define BMI270_GYRO_1000_DPS 32.8f
#define BMI270_GYRO_500_DPS 65.6f
#define BMI270_GYRO_250_DPS 131.2f
#define BMI270_GYRO_125_DPS 262.4f

#define BMI270_ACCEL_2G 16384.0f
#define BMI270_ACCEL_4G 8192.0f
#define BMI270_ACCEL_8G 4096.0f
#define BMI270_ACCEL_16G 2048.0f

#define BMI270_GRAVITY_EARTH 9.80665f

extern uint8_t buff[512];
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;

void bmi270_i2c_init(void);
void bmi270_spi_init(void);
void bmi270_spi_init_check(void);
void bmi270_print(uint16_t code);
void bmi270_pwr_conf(uint8_t pwr_mode);
const char* bmi270_codeToStr(uint16_t code);
uint8_t bmi270_spi_read_8(uint8_t reg);
uint16_t bmi270_read_gyro(uint8_t axis);
uint16_t bmi270_read_accel(uint8_t axis);
float bmi270_getGyroRange(void);
float bmi270_getAccelRange(void);
float bmi270_lsb_to_dps(int16_t val, float dps);
float bmi270_lsb_to_mps2(int16_t val, float acc_range);
void bmi270_spi_write_8(uint8_t reg, uint8_t data);
void bmi270_spi_write_burst(uint8_t reg, uint8_t data[], uint16_t data_size);

#endif /* INC_BMI270_H_ */
