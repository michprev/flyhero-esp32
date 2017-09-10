/*
 * MPU6050.cpp
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#include "MPU6050.h"

namespace flyhero {

extern "C" void int_handler(void *arg) {
        /*if (run) {
            timeval ticks;
            gettimeofday(&ticks, NULL);

            xQueueSendFromISR(int_handle, &ticks.tv_usec, NULL);
        }*/
}

MPU6050& MPU6050::Instance() {
	static MPU6050 instance;

	return instance;
}

MPU6050::MPU6050()
	: accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 10)
	, accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 10)
	, accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 10)
	, gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 60)
	, gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 60)
	, gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 60)
{
	this->g_fsr = GYRO_FSR_NOT_SET;
	this->g_mult = 0;
	this->a_mult = 0;
	this->a_fsr = ACCEL_FSR_NOT_SET;
	this->lpf = LPF_NOT_SET;
	this->sample_rate = -1;
	this->start_ticks = 0;
	this->data_ready_ticks = 0;
	this->delta_t = 0;
	this->Data_Ready_Callback = NULL;
	this->Data_Read_Callback = NULL;
}

esp_err_t MPU6050::i2c_init() {
	esp_err_t state;

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_18;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = GPIO_NUM_19;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = 400000;

	if ((state = i2c_param_config(I2C_NUM_0, &conf)))
		return state;
	if ((state = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0)))
		return state;

	return ESP_OK;
}

esp_err_t MPU6050::int_init() {
	esp_err_t state;

	gpio_config_t conf;

	conf.pin_bit_mask = GPIO_SEL_5;
	conf.mode = GPIO_MODE_INPUT;
	conf.pull_up_en = GPIO_PULLUP_DISABLE;
	conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	conf.intr_type = GPIO_INTR_POSEDGE;

	if ((state = gpio_config(&conf)))
		return state;

	if ((state = gpio_install_isr_service(0)))
		return state;
	if ((state = gpio_isr_handler_add(GPIO_NUM_5, int_handler, NULL)))
		return state;

	return ESP_OK;
}

esp_err_t MPU6050::Init() {
	uint8_t tmp;

	// init I2C bus including DMA peripheral
	if (this->i2c_init())
		return ESP_FAIL;

	// init INT pin on ESP32
	if (this->int_init())
		return ESP_FAIL;

	// reset device
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x80))
		return ESP_FAIL;

	// wait until reset done
	do {
		this->i2c_read(this->REGISTERS.PWR_MGMT_1, &tmp);
	} while (tmp & 0x80);

	// reset analog devices - should not be needed
	if (this->i2c_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07))
		return ESP_FAIL;
	vTaskDelay(100 / portTICK_RATE_MS);

	// wake up, set clock source PLL with Z gyro axis
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x03))
		return ESP_FAIL;

	vTaskDelay(50 / portTICK_RATE_MS);

	// do not disable any sensor
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_2, 0x00))
		return ESP_FAIL;

	// check I2C connection
	uint8_t who_am_i;
	if (this->i2c_read(this->REGISTERS.WHO_AM_I, &who_am_i))
		return ESP_FAIL;

	if (who_am_i != 0x68)
		return ESP_FAIL;

	// disable interrupt
	if (this->set_interrupt(false))
		return ESP_FAIL;

	// set INT pin active high, push-pull; don't use latched mode, fsync nor I2C master aux
	if (this->i2c_write(this->REGISTERS.INT_PIN_CFG, 0x00))
		return ESP_FAIL;

	// disable I2C master aux
	if (this->i2c_write(this->REGISTERS.USER_CTRL, 0x20))
		return ESP_FAIL;

	// set gyro full scale range
	if (this->set_gyro_fsr(GYRO_FSR_2000))
		return ESP_FAIL;

	// set accel full scale range
	if (this->set_accel_fsr(ACCEL_FSR_16))
		return ESP_FAIL;

	// set low pass filter to 188 Hz (both acc and gyro sample at 1 kHz) TODO
	if (this->set_lpf(LPF_188HZ))
		return ESP_FAIL;

	// set sample rate to 1 kHz
	if (this->set_sample_rate(1000))
		return ESP_FAIL;

	if (this->set_interrupt(true))
		return ESP_FAIL;

	//this->start_ticks = Timer::Get_Tick_Count(); TODO

	return ESP_OK;
}

esp_err_t MPU6050::set_gyro_fsr(gyro_fsr fsr) {
	if (this->g_fsr == fsr)
		return ESP_OK;

	if (this->i2c_write(this->REGISTERS.GYRO_CONFIG, fsr) == ESP_OK) {
		this->g_fsr = fsr;

		switch (this->g_fsr) {
		case GYRO_FSR_250:
			this->g_mult = 250;
			break;
		case GYRO_FSR_500:
			this->g_mult = 500;
			break;
		case GYRO_FSR_1000:
			this->g_mult = 1000;
			break;
		case GYRO_FSR_2000:
			this->g_mult = 2000;
			break;
		case GYRO_FSR_NOT_SET:
			return ESP_FAIL;
		}

		this->g_mult /= std::pow(2, this->ADC_BITS - 1);

		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t MPU6050::set_accel_fsr(accel_fsr fsr) {
	if (this->a_fsr == fsr)
		return ESP_OK;

	if (this->i2c_write(this->REGISTERS.ACCEL_CONFIG, fsr) == ESP_OK) {
		this->a_fsr = fsr;

		switch (this->a_fsr) {
		case ACCEL_FSR_2:
			this->a_mult = 2;
			break;
		case ACCEL_FSR_4:
			this->a_mult = 4;
			break;
		case ACCEL_FSR_8:
			this->a_mult = 8;
			break;
		case ACCEL_FSR_16:
			this->a_mult = 16;
			break;
		case ACCEL_FSR_NOT_SET:
			return ESP_FAIL;
		}

		this->a_mult /= std::pow(2, this->ADC_BITS - 1);

		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t MPU6050::set_lpf(lpf_bandwidth lpf) {
	if (this->lpf == lpf)
		return ESP_OK;

	if (this->i2c_write(this->REGISTERS.CONFIG, lpf) == ESP_OK) {
		this->lpf = lpf;
		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t MPU6050::set_sample_rate(uint16_t rate) {
	if (this->sample_rate == rate)
		return ESP_OK;

	uint8_t val = (1000 - rate) / rate;

	if (this->i2c_write(this->REGISTERS.SMPRT_DIV, val) == ESP_OK) {
		this->sample_rate = rate;
		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t MPU6050::set_interrupt(bool enable) {
	return this->i2c_write(this->REGISTERS.INT_ENABLE, enable ? 0x01 : 0x00);
}

esp_err_t MPU6050::i2c_write(uint8_t reg, uint8_t data) {
	esp_err_t state;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
		return state;
	if ((state = i2c_master_write_byte(cmd, reg, true)))
		return state;
	if ((state = i2c_master_write_byte(cmd, data, true)))
		return state;
	if ((state = i2c_master_stop(cmd)))
		return state;

	if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
		return state;

	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

esp_err_t MPU6050::i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size) {
	esp_err_t state;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
		return state;
	if ((state = i2c_master_write_byte(cmd, reg, true)))
		return state;
	if ((state = i2c_master_write(cmd, data, data_size, true)))
		return state;
	if ((state = i2c_master_stop(cmd)))
		return state;

	if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
		return state;

	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data) {
	esp_err_t state;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
		return state;
	if ((state = i2c_master_write_byte(cmd, reg, true)))
		return state;

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
		return state;
	if ((state = i2c_master_read_byte(cmd, data, 0x01)))
		return state;
	if ((state = i2c_master_stop(cmd)))
		return state;

	if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
		return state;

	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size) {
	esp_err_t state;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
		return state;
	if ((state = i2c_master_write_byte(cmd, reg, true)))
		return state;

	if ((state = i2c_master_start(cmd)))
		return state;
	if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
		return state;

	/*if ((state = i2c_master_read(cmd, data, data_size, 0x01)))
		return state;*/

	for (uint8_t i = 0; i < data_size - 1; i++) {
		if ((state = i2c_master_read_byte(cmd, data + i, 0x00)))
			return state;
	}
	if ((state = i2c_master_read_byte(cmd, data + data_size - 1, 0x01)))
		return state;

	if ((state = i2c_master_stop(cmd)))
		return state;

	if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
		return state;

	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

/*HAL_StatusTypeDef MPU6050::Start_Read() {
	if (this->data_ready_ticks != 0)
		this->delta_t = (Timer::Get_Tick_Count() - this->data_ready_ticks) * 0.000001;
	this->data_ready_ticks = Timer::Get_Tick_Count();

	return HAL_I2C_Mem_Read_DMA(&this->hi2c, this->I2C_ADDRESS, this->REGISTERS.ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, this->data_buffer, 14);
}*/

void MPU6050::Complete_Read() {
	this->raw_accel.x = (this->data_buffer[0] << 8) | this->data_buffer[1];
	this->raw_accel.y = (this->data_buffer[2] << 8) | this->data_buffer[3];
	this->raw_accel.z = (this->data_buffer[4] << 8) | this->data_buffer[5];

	this->raw_temp = (this->data_buffer[6] << 8) | this->data_buffer[7];

	this->raw_gyro.x = (this->data_buffer[8] << 8) | this->data_buffer[9];
	this->raw_gyro.y = (this->data_buffer[10] << 8) | this->data_buffer[11];
	this->raw_gyro.z = (this->data_buffer[12] << 8) | this->data_buffer[13];

	this->accel.x = this->accel_x_filter.Apply_Filter((this->raw_accel.x + this->accel_offsets[0]) * this->a_mult);
	this->accel.y = this->accel_y_filter.Apply_Filter((this->raw_accel.y + this->accel_offsets[1]) * this->a_mult);
	this->accel.z = this->accel_z_filter.Apply_Filter((this->raw_accel.z + this->accel_offsets[2]) * this->a_mult);

	this->gyro.x = this->gyro_x_filter.Apply_Filter((this->raw_gyro.x + this->gyro_offsets[0]) * this->g_mult);
	this->gyro.y = this->gyro_y_filter.Apply_Filter((this->raw_gyro.y + this->gyro_offsets[1]) * this->g_mult);
	this->gyro.z = this->gyro_z_filter.Apply_Filter((this->raw_gyro.z + this->gyro_offsets[2]) * this->g_mult);
}

void MPU6050::Get_Raw_Accel(Raw_Data& raw_accel) {
	raw_accel = this->raw_accel;
}

void MPU6050::Get_Raw_Gyro(Raw_Data& raw_gyro) {
	raw_gyro = this->raw_gyro;
}

void MPU6050::Get_Raw_Temp(int16_t& raw_temp) {
	raw_temp = this->raw_temp;
}

void MPU6050::Get_Accel(Sensor_Data& accel) {
	accel = this->accel;
}

void MPU6050::Get_Gyro(Sensor_Data& gyro) {
	gyro = this->gyro;
}

esp_err_t MPU6050::Read_Raw(Raw_Data& accel, Raw_Data& gyro) {
	uint8_t tmp[14];

	if (this->i2c_read(this->REGISTERS.ACCEL_XOUT_H, tmp, 14))
		return ESP_FAIL;

	accel.x = (tmp[0] << 8) | tmp[1];
	accel.y = (tmp[2] << 8) | tmp[3];
	accel.z = (tmp[4] << 8) | tmp[5];

	gyro.x = (tmp[8] << 8) | tmp[9];
	gyro.y = (tmp[10] << 8) | tmp[11];
	gyro.z = (tmp[12] << 8) | tmp[13];

	return ESP_OK;
}

esp_err_t MPU6050::Calibrate() {
	gyro_fsr prev_g_fsr = this->g_fsr;
	accel_fsr prev_a_fsr = this->a_fsr;

	this->set_gyro_fsr(GYRO_FSR_1000);
	this->set_accel_fsr(ACCEL_FSR_16);

	// wait until internal sensor calibration done
	//while (Timer::Get_Tick_Count() - this->start_ticks < 40000000);

	uint8_t offset_data[6] = { 0x00 };

	// gyro offsets should be already zeroed
	// for accel we need to read factory values and preserve bit 0 of LSB for each axis
	// http://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers
	this->i2c_read(this->REGISTERS.ACCEL_X_OFFSET, offset_data, 6);

	int16_t accel_offsets[3];
	accel_offsets[0] = (offset_data[0] << 8) | offset_data[1];
	accel_offsets[1] = (offset_data[2] << 8) | offset_data[3];
	accel_offsets[2] = (offset_data[4] << 8) | offset_data[5];

	int32_t offsets[6] = { 0 };
	Raw_Data gyro, accel;

	// we want accel Z to be 2048 (+ 1g)

	for (uint16_t i = 0; i < 500; i++) {
		this->Read_Raw(accel, gyro);

		offsets[0] += accel.x;
		offsets[1] += accel.y;
		offsets[2] += accel.z - 2048;
		offsets[3] += gyro.x;
		offsets[4] += gyro.y;
		offsets[5] += gyro.z;
	}

	int16_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
	accel_x = -offsets[0] / 500;
	accel_y = -offsets[1] / 500;
	accel_z = -offsets[2] / 500;
	gyro_x = -offsets[3] / 500;
	gyro_y = -offsets[4] / 500;
	gyro_z = -offsets[5] / 500;

	accel_offsets[0] += accel_x;
	accel_offsets[1] += accel_y;
	accel_offsets[2] += accel_z;

	if (offset_data[1] & 0x01)
		accel_offsets[0] |= 0x01;
	else
		accel_offsets[0] &= 0xFFFE;

	if (offset_data[3] & 0x01)
		accel_offsets[1] |= 0x01;
	else
		accel_offsets[1] &= 0xFFFE;

	if (offset_data[5] & 0x01)
		accel_offsets[2] |= 0x01;
	else
		accel_offsets[2] &= 0xFFFE;

	offset_data[0] = accel_offsets[0] >> 8;
	offset_data[1] = accel_offsets[0] & 0xFF;
	offset_data[2] = accel_offsets[1] >> 8;
	offset_data[3] = accel_offsets[1] & 0xFF;
	offset_data[4] = accel_offsets[2] >> 8;
	offset_data[5] = accel_offsets[2] & 0xFF;

	this->i2c_write(this->REGISTERS.ACCEL_X_OFFSET, offset_data, 6);

	offset_data[0] = gyro_x >> 8;
	offset_data[1] = gyro_x & 0xFF;
	offset_data[2] = gyro_y >> 8;
	offset_data[3] = gyro_y & 0xFF;
	offset_data[4] = gyro_z >> 8;
	offset_data[5] = gyro_z & 0xFF;

	this->i2c_write(this->REGISTERS.GYRO_X_OFFSET, offset_data, 6);

	// set gyro & accel FSR to its original value
	this->set_gyro_fsr(prev_g_fsr);
	this->set_accel_fsr(prev_a_fsr);

	vTaskDelay(5000 / portTICK_RATE_MS);

	// lets measure offsets again to be applied on STM
	this->accel_offsets[0] = this->accel_offsets[1] = this->accel_offsets[2] = 0;
	this->gyro_offsets[0] = this->gyro_offsets[1] = this->gyro_offsets[2] = 0;

	for (uint16_t i = 0; i < 500; i++) {
		this->Read_Raw(accel, gyro);

		this->accel_offsets[0] += accel.x;
		this->accel_offsets[1] += accel.y;
		this->accel_offsets[2] += accel.z - 2048;
		this->gyro_offsets[0] += gyro.x;
		this->gyro_offsets[1] += gyro.y;
		this->gyro_offsets[2] += gyro.z;
	}

	this->accel_offsets[0] /= -500;
	this->accel_offsets[1] /= -500;
	this->accel_offsets[2] /= -500;
	this->gyro_offsets[0] /= -500;
	this->gyro_offsets[1] /= -500;
	this->gyro_offsets[2] /= -500;

	return ESP_OK;
}

inline float MPU6050::inv_sqrt(float x) {
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (0.5f * x * y * y));

	return y;
}

// use Betaflight atan2 approx: https://github.com/betaflight/betaflight/blob/master/src/main/common/maths.c
float MPU6050::atan2(float y, float x) {
	const float atanPolyCoef1 = 3.14551665884836e-07f;
	const float atanPolyCoef2 = 0.99997356613987f;
	const float atanPolyCoef3 = 0.14744007058297684f;
	const float atanPolyCoef4 = 0.3099814292351353f;
	const float atanPolyCoef5 = 0.05030176425872175f;
	const float atanPolyCoef6 = 0.1471039133652469f;
	const float atanPolyCoef7 = 0.6444640676891548f;

	float abs_x, abs_y;
	float result;

	abs_x = std::abs(x);
	abs_y = std::abs(y);

	result = (abs_x > abs_y ? abs_x : abs_y);

	if (result != 0)
		result = (abs_x < abs_y ? abs_x : abs_y) / result;

	result = -((((atanPolyCoef5 * result - atanPolyCoef4) * result - atanPolyCoef3) * result - atanPolyCoef2) * result - atanPolyCoef1) / ((atanPolyCoef7 * result + atanPolyCoef6) * result + 1.0);
	result *= this->RAD_TO_DEG;

	if (abs_y > abs_x)
		result = 90 - result;
	if (x < 0)
		result = 180 - result;
	if (y < 0)
		result = -result;

	return result;
}

// approx. using http://nghiaho.com/?p=997
inline double MPU6050::atan(double z) {
	if (z < 0)
		return -this->atan(-z);

	if (z > 1)
		return 90 - this->atan(1 / z);

	return z * (45 - (z - 1) * (14 + 3.83 * z));
}

} /* namespace The_Eye */
