/*
 * MPU6050.cpp
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#include <MPU6050.h>

namespace The_Eye {

MPU6050* MPU6050::pInstance = NULL;

MPU6050* MPU6050::Instance() {
	if (MPU6050::pInstance == NULL)
		pInstance = new MPU6050();

	return pInstance;
}

MPU6050::MPU6050() {
	this->g_fsr = GYRO_FSR_NOT_SET;
	this->a_fsr = ACCEL_FSR_NOT_SET;
	this->lpf = LPF_NOT_SET;
}

HAL_StatusTypeDef MPU6050::Init(I2C_HandleTypeDef *hi2c, bool use_dmp) {
	this->use_DMP = use_dmp;

	// reset device
	this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x80);
	HAL_Delay(100);

	// reset analog devices
	this->i2c_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07);
	HAL_Delay(100);

	// wake up
	this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x00);

	// set gyro full scale range
	this->set_gyro_fsr(GYRO_FSR_2000);

	// set accel full scale range
	this->set_accel_fsr(ACCEL_FSR_2);

	// set low pass filter to 42 Hz
	this->set_lpf(LPF_42HZ);
}

HAL_StatusTypeDef MPU6050::set_gyro_fsr(gyro_fsr fsr) {
	if (this->g_fsr == fsr)
		return HAL_OK;

	if (this->i2c_write(this->REGISTERS.GYRO_CONFIG, fsr) == HAL_OK) {
		this->g_fsr = fsr;
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::set_accel_fsr(accel_fsr fsr) {
	if (this->a_fsr == fsr)
		return HAL_OK;

	if (this->i2c_write(this->REGISTERS.ACCEL_CONFIG, fsr) == HAL_OK) {
		this->a_fsr = fsr;
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::set_lpf(lpf_bandwidth lpf) {
	if (this->lpf == lpf)
		return HAL_OK;

	if (this->i2c_write(this->REGISTERS.CONFIG, lpf) == HAL_OK) {
		this->lpf = lpf;
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::i2c_write(uint8_t reg, uint8_t val) {
	return HAL_I2C_Mem_Write(&this->hi2c, this->ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU6050::Calibrate() {

}

} /* namespace The_Eye */
