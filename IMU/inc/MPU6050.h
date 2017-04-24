/*
 * MPU6050.h
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include <stm32f4xx_hal.h>

namespace The_Eye {

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

const uint8_t ADDRESS = 0x68;
const uint16_t I2C_TIMEOUT = 500;
const struct {
	uint8_t CONFIG = 0x1A;
	uint8_t GYRO_CONFIG = 0x1B;
	uint8_t ACCEL_CONFIG = 0x1C;
	uint8_t SIGNAL_PATH_RESET = 0x68;
	uint8_t PWR_MGMT_1 = 0x6B;
} REGISTERS;

bool use_DMP;
I2C_HandleTypeDef hi2c;
gyro_fsr g_fsr; // TODO better variable/enum name
accel_fsr a_fsr;
lpf_bandwidth lpf;

HAL_StatusTypeDef i2c_write(uint8_t reg, uint8_t val);
HAL_StatusTypeDef set_gyro_fsr(gyro_fsr fsr);
HAL_StatusTypeDef set_accel_fsr(accel_fsr fsr);
HAL_StatusTypeDef set_lpf(lpf_bandwidth lpf);

public:
	struct Sensor_Data {
		double x, y, z;
	};

	static MPU6050* Instance();

	// TODO as of board 2.0 use special SPI bus for IMU
	HAL_StatusTypeDef Init(I2C_HandleTypeDef *hi2c, bool use_DMP);
	HAL_StatusTypeDef Calibrate();
};

} /* namespace The_Eye */

#endif /* MPU6050_H_ */
