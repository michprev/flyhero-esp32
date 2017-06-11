/*
 * MPU6050.h
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include <math.h>
#include <stm32f4xx_hal.h>
#include "Timer.h"

//#include "invensense/library/fast_no_motion.h"

namespace flyhero {

class MPU6050 {
private:
	/* Singleton begin */
	MPU6050();
	MPU6050(MPU6050 const&){};
	MPU6050& operator=(MPU6050 const&){};
	static MPU6050* pInstance;
	/*Singleton end */

enum gyro_fsr {
	GYRO_FSR_250 = 0x00,
	GYRO_FSR_500 = 0x08,
	GYRO_FSR_1000 = 0x10,
	GYRO_FSR_2000 = 0x18,
	GYRO_FSR_NOT_SET = 0xFF
};

enum accel_fsr {
	ACCEL_FSR_2 = 0x00,
	ACCEL_FSR_4 = 0x08,
	ACCEL_FSR_8 = 0x10,
	ACCEL_FSR_16 = 0x18,
	ACCEL_FSR_NOT_SET = 0xFF
};

enum lpf_bandwidth {
	LPF_256HZ = 0x00,
	LPF_188HZ = 0x01,
	LPF_98HZ = 0x02,
	LPF_42HZ = 0x03,
	LPF_20HZ = 0x04,
	LPF_10HZ = 0x05,
	LPF_5HZ = 0x06,
	LPF_NOT_SET = 0xFF
};

const double PI = 3.14159265358979323846;
const uint8_t ADC_BITS = 16;
const uint8_t I2C_ADDRESS = 0xD0;
const uint16_t I2C_TIMEOUT = 500;
const uint16_t DMP_START_ADDRESS = 0x0400;
static const uint16_t DMP_FIRMWARE_SIZE = 3062;
static const uint8_t DMP_FIRMWARE[DMP_FIRMWARE_SIZE];

const struct {
	uint8_t ACCEL_X_OFFSET = 0x06;
	uint8_t GYRO_X_OFFSET = 0x13;
	uint8_t SMPRT_DIV = 0x19;
	uint8_t CONFIG = 0x1A;
	uint8_t GYRO_CONFIG = 0x1B;
	uint8_t ACCEL_CONFIG = 0x1C;
	uint8_t FIFO_EN = 0x23;
	uint8_t INT_PIN_CFG = 0x37;
	uint8_t INT_ENABLE = 0x38;
	uint8_t INT_STATUS = 0x3A;
	uint8_t ACCEL_XOUT_H = 0x3B;
	uint8_t SIGNAL_PATH_RESET = 0x68;
	uint8_t USER_CTRL = 0x6A;
	uint8_t PWR_MGMT_1 = 0x6B;
	uint8_t PWR_MGMT_2 = 0x6C;
	uint8_t BANK_SEL = 0x6D;
	uint8_t FIFO_COUNT_H = 0x72;
	uint8_t FIFO_COUNT_L = 0x73;
	uint8_t FIFO_R_W = 0x74;
	uint8_t WHO_AM_I = 0x75;
} REGISTERS;

uint32_t start_ticks;
double roll, pitch, yaw;
bool use_DMP;
I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma_i2c_rx;
gyro_fsr g_fsr; // TODO better variable/enum name
double g_div;
double a_div;
accel_fsr a_fsr;
lpf_bandwidth lpf;
int16_t sample_rate;
uint16_t data_size;
uint8_t data_buffer[14];
volatile bool data_ready;
volatile bool data_read;
volatile uint32_t data_ready_ticks;
volatile double deltaT;

HAL_StatusTypeDef i2c_init();
void int_init();
HAL_StatusTypeDef i2c_write(uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size);
HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size);
HAL_StatusTypeDef set_gyro_fsr(gyro_fsr fsr);
HAL_StatusTypeDef set_accel_fsr(accel_fsr fsr);
HAL_StatusTypeDef set_lpf(lpf_bandwidth lpf);
HAL_StatusTypeDef set_sample_rate(uint16_t rate);
HAL_StatusTypeDef set_interrupt(bool enable);
HAL_StatusTypeDef reset_fifo();
HAL_StatusTypeDef load_DMP_firmware();

public:
	struct Sensor_Data {
		double x, y, z;
	};

	struct Raw_Data {
		int16_t x, y, z;
	};

	enum Debug_Enum { DATA_READY_INTERRUPT, DATA_START_READ, DATA_READ_INTERRUPT };

	struct Debug_Data {
		Debug_Enum state;
		uint32_t ticks;
		uint32_t delta;
	};

	Debug_Data ddata[1000];
	uint16_t ddata_pos = 0;
	bool ready;

	static MPU6050* Instance();

	void Data_Ready_Callback();
	bool Data_Ready();
	void Data_Read_Callback();
	bool Data_Read();

	DMA_HandleTypeDef* Get_DMA_Rx_Handle();
	I2C_HandleTypeDef* Get_I2C_Handle();

	HAL_StatusTypeDef Init(bool use_DMP);
	HAL_StatusTypeDef Read_FIFO();
	HAL_StatusTypeDef Parse_FIFO();
	HAL_StatusTypeDef Calibrate();
	bool FIFO_Overflow();
	HAL_StatusTypeDef Get_Euler(double *roll, double *pitch, double *yaw);
	HAL_StatusTypeDef Start_Read_Raw();
	HAL_StatusTypeDef Complete_Read_Raw(Raw_Data *gyro, Raw_Data *accel);
	HAL_StatusTypeDef Read_Raw(Raw_Data *gyro, Raw_Data *accel);
};

} /* namespace The_Eye */

#endif /* MPU6050_H_ */
