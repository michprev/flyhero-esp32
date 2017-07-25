/*
 * MPU6050.h
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include <cmath>
#include <stm32f4xx_hal.h>
#include "Timer.h"
#include "Biquad_Filter.h"
#include "LEDs.h"

namespace flyhero {

class MPU6050 {
public:
	struct Sensor_Data {
		float x, y, z;
	};

	struct Raw_Data {
		int16_t x, y, z;
	};

	struct Quaternion {
		float q0, q1, q2, q3;
	};

private:
	/* Singleton begin */
	MPU6050();
	MPU6050(MPU6050 const&)
	: accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	{};
	MPU6050& operator=(MPU6050 const&){};
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

const float COMPLEMENTARY_COEFFICIENT = 0.995f;
const double PI = 3.14159265358979323846;
const float RAD_TO_DEG = 180 / this->PI;
const float DEG_TO_RAD = this->PI / 180;
const uint8_t ADC_BITS = 16;
const uint8_t I2C_ADDRESS = 0xD0;
const uint16_t I2C_TIMEOUT = 500;

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
	uint8_t FIFO_COUNT_H = 0x72;
	uint8_t FIFO_COUNT_L = 0x73;
	uint8_t FIFO_R_W = 0x74;
	uint8_t WHO_AM_I = 0x75;
} REGISTERS;

Biquad_Filter accel_x_filter, accel_y_filter, accel_z_filter;
Biquad_Filter gyro_x_filter, gyro_y_filter, gyro_z_filter;

Sensor_Data accel, gyro;
Sensor_Data mahony_integral;
Quaternion quaternion;
float mahony_Kp, mahony_Ki;
int16_t raw_temp;
Raw_Data raw_accel, raw_gyro;
uint32_t start_ticks;
float roll, pitch, yaw;
I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma_i2c_rx;
gyro_fsr g_fsr; // TODO better variable/enum name
float g_mult;
float a_mult;
accel_fsr a_fsr;
lpf_bandwidth lpf;
int16_t sample_rate;
uint8_t data_buffer[14];
float accel_offsets[3];
float gyro_offsets[3];
volatile uint32_t data_ready_ticks;
volatile float delta_t;

float atan2(float y, float x);
inline float inv_sqrt(float x);
inline double atan(double z);

void i2c_reset_bus();
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

public:
	static MPU6050& Instance();

	void (*Data_Ready_Callback)();
	void (*Data_Read_Callback)();

	DMA_HandleTypeDef* Get_DMA_Rx_Handle();
	I2C_HandleTypeDef* Get_I2C_Handle();

	void Reset_Integrators();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Calibrate();
	void Compute_Euler();
	void Compute_Mahony();
	void Get_Euler(float& roll, float& pitch, float& yaw);
	void Get_Raw_Accel(Raw_Data& raw_accel);
	void Get_Raw_Gyro(Raw_Data& raw_gyro);
	void Get_Raw_Temp(int16_t& raw_temp);
	void Get_Accel(Sensor_Data& accel);
	void Get_Gyro(Sensor_Data& gyro);
	HAL_StatusTypeDef Read_Raw(Raw_Data& accel, Raw_Data& gyro);
	HAL_StatusTypeDef Start_Read();
	void Complete_Read();
};

} /* namespace The_Eye */

#endif /* MPU6050_H_ */
