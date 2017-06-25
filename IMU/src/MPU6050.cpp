/*
 * MPU6050.cpp
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#include <MPU6050.h>

namespace flyhero {

extern "C" {
	void DMA1_Stream5_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(MPU6050::Instance().Get_DMA_Rx_Handle());
	}

	void EXTI1_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	}

	void I2C1_EV_IRQHandler(void)
	{
		HAL_I2C_EV_IRQHandler(MPU6050::Instance().Get_I2C_Handle());
	}

	void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
		MPU6050::Instance().Data_Read_Callback();
	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if (MPU6050::Instance().Start_Read_Raw() != HAL_OK)
			LEDs::TurnOn(LEDs::Orange);
	}
}

MPU6050& MPU6050::Instance() {
	static MPU6050 instance;

	return instance;
}

MPU6050::MPU6050()
	: accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
	, gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, 1000, 50)
{
	this->g_fsr = GYRO_FSR_NOT_SET;
	this->g_mult = 0;
	this->a_mult = 0;
	this->a_fsr = ACCEL_FSR_NOT_SET;
	this->lpf = LPF_NOT_SET;
	this->sample_rate = -1;
	this->ready = false;
	this->data_read = false;
	this->roll = 0;
	this->pitch = 0;
	this->yaw = 0;
	this->start_ticks = 0;
	this->data_ready_ticks = 0;
	this->delta_t = 0;
}

DMA_HandleTypeDef* MPU6050::Get_DMA_Rx_Handle() {
	return &this->hdma_i2c_rx;
}

I2C_HandleTypeDef* MPU6050::Get_I2C_Handle() {
	return &this->hi2c;
}

void MPU6050::Data_Read_Callback() {
	this->data_read = true;
}

bool MPU6050::Data_Read() {
	return this->data_read;
}

// Bug solved here: https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour
// STMicroelectronics errata: http://www.st.com/content/ccc/resource/technical/document/errata_sheet/7f/05/b0/bc/34/2f/4c/21/CD00288116.pdf/files/CD00288116.pdf/jcr:content/translations/en.CD00288116.pdf

void MPU6050::i2c_reset_bus() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register
	I2C1->CR1 &= ~(0x0001);

	// 2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR)
	GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull         = GPIO_PULLUP;
	GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStructure.Pin			= GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	GPIO_InitStructure.Pin			= GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	// 3. Check SCL and SDA High level in GPIOx_IDR
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_SET &&
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) != GPIO_PIN_SET) {
	}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

	// 5. Check SDA Low level in GPIOx_IDR
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) != GPIO_PIN_RESET) {
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

	// 7. Check SCL Low level in GPIOx_IDR
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_RESET) {
	}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_SET) {
	}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR
	// this one never worked so just skip it
	/*while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) != GPIO_PIN_SET) {
	}*/

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain
	GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;

	GPIO_InitStructure.Pin          = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin          = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 13. Set SWRST bit in I2Cx_CR1 register
	I2C1->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register
	I2C1->CR1 &= ~(0x8000);

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	I2C1->CR1 |= 0x0001;


}

HAL_StatusTypeDef MPU6050::i2c_init() {
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();
	if (__I2C1_IS_CLK_DISABLED())
		__I2C1_CLK_ENABLE();
	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	this->i2c_reset_bus();

	this->hi2c.Instance = I2C1;
	this->hi2c.Init.ClockSpeed = 400000;
	this->hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	this->hi2c.Init.OwnAddress1 = 0;
	this->hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	this->hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	this->hi2c.Init.OwnAddress2 = 0;
	this->hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	this->hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&this->hi2c);

	this->hdma_i2c_rx.Instance = DMA1_Stream5;
	this->hdma_i2c_rx.Init.Channel = DMA_CHANNEL_1;
	this->hdma_i2c_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	this->hdma_i2c_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	this->hdma_i2c_rx.Init.MemInc = DMA_MINC_ENABLE;
	this->hdma_i2c_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	this->hdma_i2c_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	this->hdma_i2c_rx.Init.Mode = DMA_NORMAL;
	this->hdma_i2c_rx.Init.Priority = DMA_PRIORITY_LOW;
	this->hdma_i2c_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&this->hdma_i2c_rx);

	__HAL_LINKDMA(&this->hi2c, hdmarx, this->hdma_i2c_rx);

	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	return HAL_OK;
}

void MPU6050::int_init() {
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef exti;

	exti.Pin = GPIO_PIN_1;
	exti.Mode = GPIO_MODE_IT_RISING;
	exti.Pull = GPIO_NOPULL;
	exti.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &exti);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

HAL_StatusTypeDef MPU6050::Init() {
	uint8_t tmp;

	// init I2C bus including DMA peripheral
	if (this->i2c_init())
		return HAL_ERROR;

	// init INT pin on STM32
	this->int_init();

	// reset device
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x80))
		return HAL_ERROR;

	// wait until reset done
	do {
		this->i2c_read(this->REGISTERS.PWR_MGMT_1, &tmp);
	} while (tmp & 0x80);

	// reset analog devices - should not be needed
	if (this->i2c_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07))
		return HAL_ERROR;
	HAL_Delay(100);

	// wake up, set clock source PLL with Z gyro axis
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x03))
		return HAL_ERROR;

	HAL_Delay(50);

	// do not disable any sensor
	if (this->i2c_write(this->REGISTERS.PWR_MGMT_2, 0x00))
		return HAL_ERROR;

	// check I2C connection
	uint8_t who_am_i;
	if (this->i2c_read(this->REGISTERS.WHO_AM_I, &who_am_i))
		return HAL_ERROR;

	if (who_am_i != 0x68)
		return HAL_ERROR;

	// disable interrupt
	if (this->set_interrupt(false))
		return HAL_ERROR;

	// set INT pin active high, push-pull; don't use latched mode, fsync nor I2C master aux
	if (this->i2c_write(this->REGISTERS.INT_PIN_CFG, 0x00))
		return HAL_ERROR;

	// disable I2C master aux
	if (this->i2c_write(this->REGISTERS.USER_CTRL, 0x20))
		return HAL_ERROR;

	// set gyro full scale range
	if (this->set_gyro_fsr(GYRO_FSR_2000))
		return HAL_ERROR;

	// set accel full scale range
	if (this->set_accel_fsr(ACCEL_FSR_2))
		return HAL_ERROR;

	// set low pass filter to 188 Hz (both acc and gyro sample at 1 kHz)
	if (this->set_lpf(LPF_188HZ))
		return HAL_ERROR;

	// set sample rate to 1 kHz
	if (this->set_sample_rate(1000))
		return HAL_ERROR;

	if (this->set_interrupt(true))
		return HAL_ERROR;

	this->start_ticks = Timer::Get_Tick_Count();

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050::set_gyro_fsr(gyro_fsr fsr) {
	if (this->g_fsr == fsr)
		return HAL_OK;

	if (this->i2c_write(this->REGISTERS.GYRO_CONFIG, fsr) == HAL_OK) {
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
		}

		this->g_mult /= pow(2, this->ADC_BITS - 1);

		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::set_accel_fsr(accel_fsr fsr) {
	if (this->a_fsr == fsr)
		return HAL_OK;

	if (this->i2c_write(this->REGISTERS.ACCEL_CONFIG, fsr) == HAL_OK) {
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
		}

		this->a_mult /= pow(2, this->ADC_BITS - 1);

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

HAL_StatusTypeDef MPU6050::set_sample_rate(uint16_t rate) {
	if (this->sample_rate == rate)
		return HAL_OK;

	uint8_t val = (1000 - rate) / rate;

	if (this->i2c_write(this->REGISTERS.SMPRT_DIV, val) == HAL_OK) {
		this->sample_rate = rate;
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::set_interrupt(bool enable) {
	return this->i2c_write(this->REGISTERS.INT_ENABLE, enable ? 0x01 : 0x00);
}

HAL_StatusTypeDef MPU6050::i2c_write(uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(&this->hi2c, this->I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU6050::i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size) {
	return HAL_I2C_Mem_Write(&this->hi2c, this->I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, data_size, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU6050::i2c_read(uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read(&this->hi2c, this->I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU6050::i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size) {
	return HAL_I2C_Mem_Read(&this->hi2c, this->I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, data_size, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU6050::Start_Read_Raw() {
	if (this->ready) {
		if (this->data_ready_ticks != 0)
			this->delta_t = (Timer::Get_Tick_Count() - this->data_ready_ticks) * 0.000001;
		this->data_ready_ticks = Timer::Get_Tick_Count();

		return HAL_I2C_Mem_Read_DMA(&this->hi2c, this->I2C_ADDRESS, this->REGISTERS.ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, this->data_buffer, 14);
	}
	
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050::Complete_Read_Raw(Raw_Data *gyro, Raw_Data *accel, int16_t *temp) {
	this->data_read = false;

	accel->x = (this->data_buffer[0] << 8) | this->data_buffer[1];
	accel->y = (this->data_buffer[2] << 8) | this->data_buffer[3];
	accel->z = (this->data_buffer[4] << 8) | this->data_buffer[5];

	*temp = (this->data_buffer[6] << 8) | this->data_buffer[7];

	gyro->x = (this->data_buffer[8] << 8) | this->data_buffer[9];
	gyro->y = (this->data_buffer[10] << 8) | this->data_buffer[11];
	gyro->z = (this->data_buffer[12] << 8) | this->data_buffer[13];

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050::Read_Raw(Raw_Data *gyro, Raw_Data *accel) {
	uint8_t tmp[14];

	if (HAL_I2C_Mem_Read(&this->hi2c, this->I2C_ADDRESS, this->REGISTERS.ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, tmp, 14, this->I2C_TIMEOUT))
		return HAL_ERROR;

	accel->x = (tmp[0] << 8) | tmp[1];
	accel->y = (tmp[2] << 8) | tmp[3];
	accel->z = (tmp[4] << 8) | tmp[5];

	/*accel->x = raw_accel.x / this->a_div;
	accel->y = raw_accel.y / this->a_div;
	accel->z = raw_accel.z / this->a_div;*/

	gyro->x = (tmp[8] << 8) | tmp[9];
	gyro->y = (tmp[10] << 8) | tmp[11];
	gyro->z = (tmp[12] << 8) | tmp[13];

	/*gyro->x = raw_gyro.x / this->g_div;
	gyro->y = raw_gyro.y / this->g_div;
	gyro->z = raw_gyro.z / this->g_div;*/

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050::Calibrate() {
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
		this->Read_Raw(&gyro, &accel);

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

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050::Get_Euler(float *roll, float *pitch, float *yaw) {
	Sensor_Data gyro, accel;
	Raw_Data raw_gyro, raw_accel;

	this->data_read = false;

	raw_accel.x = (this->data_buffer[0] << 8) | this->data_buffer[1];
	raw_accel.y = (this->data_buffer[2] << 8) | this->data_buffer[3];
	raw_accel.z = (this->data_buffer[4] << 8) | this->data_buffer[5];

	raw_gyro.x = (this->data_buffer[8] << 8) | this->data_buffer[9];
	raw_gyro.y = (this->data_buffer[10] << 8) | this->data_buffer[11];
	raw_gyro.z = (this->data_buffer[12] << 8) | this->data_buffer[13];

	accel.x = this->accel_x_filter.Apply_Filter(raw_accel.x);
	accel.y = this->accel_y_filter.Apply_Filter(raw_accel.y);
	accel.z = this->accel_z_filter.Apply_Filter(raw_accel.z);

	gyro.x = this->gyro_x_filter.Apply_Filter(raw_gyro.x);
	gyro.y = this->gyro_y_filter.Apply_Filter(raw_gyro.y);
	gyro.z = this->gyro_z_filter.Apply_Filter(raw_gyro.z);

	// 70 us
	float accel_roll = this->atan2(accel.y, accel.z);

	// 70 us
	float accel_pitch = this->atan2(-accel.x, std::sqrt((float)(accel.y * accel.y + accel.z * accel.z)));

	// 13 us
	this->roll = this->COMPLEMENTARY_COEFFICIENT * (this->roll + gyro.y * this->g_mult * this->delta_t) + (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_roll;

	// 13 us
	this->pitch = this->COMPLEMENTARY_COEFFICIENT * (this->pitch + gyro.x * this->g_mult * this->delta_t) + (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_pitch;

	// 4 us
	this->yaw += gyro.z * this->g_mult * this->delta_t;

	*roll = this->roll;
	*pitch = this->pitch;
	*yaw = this->yaw;

	return HAL_OK;
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

void MPU6050::Reset_Integrators() {
	this->roll = 0;
	this->pitch = 0;
	this->yaw = 0;
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
