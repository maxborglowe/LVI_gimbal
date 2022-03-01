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
#include "def.h"

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
	#define acc_odr_0p78 0x01
	#define acc_odr_1p5 0x02
	#define acc_odr_3p1 0x03
	#define acc_odr_6p25 0x04
	#define acc_odr_12p5 0x05
	#define acc_odr_25 0x06
	#define acc_odr_50 0x07
	#define acc_odr_100 0x08
	#define acc_odr_200 0x09
	#define acc_odr_400 0x0A
	#define acc_odr_800 0x0B
	#define acc_odr_1k6 0x0C

	#define acc_osr4_avg1 0x00
	#define acc_osr2_avg2 0x01
	#define acc_norm_avg4 0x02
	#define acc_cic_avg8 0x03
	#define acc_res_avg16 0x04
	#define acc_res_avg32 0x05
	#define acc_res_avg64 0x06
	#define acc_res_avg128 0x07

	#define acc_ulp 0x00
	#define acc_hp 0x01
//########################
#define REG_ACC_RANGE 0x41
	#define gyr_odr_25 0x06
	#define gyr_odr_50 0x07
	#define gyr_odr_100 0x08
	#define gyr_odr_200 0x09
	#define gyr_odr_400 0x0A
	#define gyr_odr_800 0x0B
	#define gyr_odr_1k6 0x0C
	#define gyr_odr_3k2 0x0D

	#define range_2g 0x00
	#define range_4g 0x01
	#define range_8g 0x02
	#define range_16g 0x03
//########################
#define REG_GYR_CONF 0x42
	#define gyr_osr4 0x00
	#define gyr_osr2 0x01
	#define gyr_norm 0x02
	#define gyr_res 0x03

	#define gyr_ulp 0x00
	#define gyr_hp 0x01
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

#define BMI270_SPI 1

#define BMI270_PWR_MODE_LOW 0
#define BMI270_PWR_MODE_NORM 1
#define BMI270_PWR_MODE_PERF 2

#define BMI270_GYRO_2000_DPS 16.384f
#define BMI270_GYRO_1000_DPS 32.768f
#define BMI270_GYRO_500_DPS 65.536f
#define BMI270_GYRO_250_DPS 131.072f
#define BMI270_GYRO_125_DPS 262.144f

#define BMI270_ACCEL_2G 16384.0f
#define BMI270_ACCEL_4G 8192.0f
#define BMI270_ACCEL_8G 4096.0f
#define BMI270_ACCEL_16G 2048.0f

#define BMI270_GRAVITY_EARTH 9.80665f

/* How may cycles the gyro noise limit should be calibrated*/
#define BMI270_CALIBRATION_TIM 300



typedef struct IMU{
	float gyr_range, acc_range;

	float inv_gyr_range, inv_acc_range;

	float gyr_x, gyr_y, gyr_z;
	float acc_x, acc_y, acc_z;
	float roll, pitch, yaw;

	float gyr_lim_min_x, gyr_lim_min_y, gyr_lim_min_z;
	float gyr_lim_max_x, gyr_lim_max_y, gyr_lim_max_z;

	uint16_t calibration_c;
	/* GYR_CONF */
	uint8_t gyr_odr, gyr_bwp, gyr_noise_perf, gyr_filter_perf;
	/* ACC_CONF */
	uint8_t acc_odr, acc_bwp, acc_filter_perf;
}IMU;

extern char* buff[BUFF_SIZE];
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
void bmi270_getGyroRange(IMU *Imu);
void bmi270_getAccelRange(IMU *Imu);
void bmi270_getGyroConf(IMU *Imu);
void bmi270_setGyroConf(IMU *Imu);
void bmi270_getAccConf(IMU *Imu);
void bmi270_setAccConf(IMU *Imu);

void bmi270_calibrateInit(IMU *Imu, float lim);
void bmi270_calibrateNoise(IMU *Imu);

float bmi270_lsb_to_dps(int16_t val, float dps);
float bmi270_lsb_to_mps2(int16_t val, float acc_range);
void bmi270_spi_write_8(uint8_t reg, uint8_t data);
void bmi270_spi_write_burst(uint8_t reg, uint8_t data[], uint16_t data_size);

#endif /* INC_BMI270_H_ */
