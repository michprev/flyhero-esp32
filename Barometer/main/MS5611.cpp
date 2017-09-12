#include "MS5611.h"

namespace flyhero {

MS5611& MS5611::Instance() {
	static MS5611 instance;

	return instance;
}

MS5611::MS5611() {
	this->D1_Timestamp = 0;
	this->D2_Timestamp = 0;
	this->D1 = 0;
	this->D2 = 0;
}

HAL_StatusTypeDef MS5611::Reset()
{
	uint8_t reset = 0x1E;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(this->hi2c, this->ADDRESS, &reset, 1, HAL_MAX_DELAY);

	HAL_Delay(20);

	return status;
}

HAL_StatusTypeDef MS5611::Init(I2C_HandleTypeDef *hi2c)
{
	this->hi2c = hi2c;

	HAL_StatusTypeDef status;

	uint8_t data[2];

	for (uint8_t i = 0; i < 6; i++) {
		status = HAL_I2C_Mem_Read(this->hi2c, this->ADDRESS, 0xA2 + i * 2, I2C_MEMADD_SIZE_8BIT, data, 2, 50);

		if (status != HAL_OK)
			return status;

		C[i] = data[0] << 8;
		C[i] |= data[1];

		HAL_Delay(10);
	}

	return HAL_OK;
}

bool MS5611::D1_Ready() {
	if (this->D1_Timestamp != 0 && HAL_GetTick() - this->D1_Timestamp >= 10) {
		uint8_t conversionData[3];
		HAL_I2C_Mem_Read(this->hi2c, this->ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, conversionData, 3, HAL_MAX_DELAY);
		this->D1 = conversionData[0] << 16;
		this->D1 |= conversionData[1] << 8;
		this->D1 |= conversionData[2];

		this->D1_Timestamp = 0;

		return true;
	}

	return false;
}

bool MS5611::D2_Ready() {
	if (this->D2_Timestamp != 0 && HAL_GetTick() - this->D2_Timestamp >= 10) {
		uint8_t conversionData[3];
		HAL_I2C_Mem_Read(this->hi2c, this->ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, conversionData, 3, HAL_MAX_DELAY);
		this->D2 = conversionData[0] << 16;
		this->D2 |= conversionData[1] << 8;
		this->D2 |= conversionData[2];

		this->D2_Timestamp = 0;

		return true;
	}

	return false;
}

HAL_StatusTypeDef MS5611::ConvertD1() {
	uint8_t convertD1 = 0x48;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(this->hi2c, this->ADDRESS, &convertD1, 1, HAL_MAX_DELAY);
	this->D1_Timestamp = HAL_GetTick();

	return status;
}

HAL_StatusTypeDef MS5611::ConvertD2() {
	uint8_t convertD2 = 0x58;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(this->hi2c, this->ADDRESS, &convertD2, 1, HAL_MAX_DELAY);
	this->D2_Timestamp = HAL_GetTick();

	return status;
}

HAL_StatusTypeDef MS5611::GetData(int32_t * temperature, int32_t * pressure)
{
	int32_t dT = D2 - (uint32_t)C[4] * pow(2, 8);
	int32_t TEMP = 2000 + dT * C[5] / pow(2, 23);
	int64_t OFF = (int64_t)C[1] * pow(2, 16) + (int64_t)(C[3] * dT) / pow(2, 7);
	int64_t SENS = (int64_t)C[0] * pow(2, 15) + (int64_t)(C[2] * dT) / pow(2, 8);

	int32_t TEMP2 = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	if (TEMP < 2000) {
		TEMP2 = (dT * dT) / pow(2, 31);
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}
	if (TEMP < -1500) {
		OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	TEMP -= TEMP2;
	OFF -= OFF2;
	SENS -= SENS2;

	int32_t P = (((double)D1) * SENS / pow(2, 21) - OFF) / pow(2, 15);

	*temperature = TEMP;
	*pressure = P;

	return HAL_OK;
}

/*HAL_StatusTypeDef MS5611::I2C_Init()
{
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	if (__I2C1_IS_CLK_DISABLED())
		__I2C1_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Pull = GPIO_PULLUP;
	gpio.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
	HAL_GPIO_Init(GPIOB, &gpio);

	this->hi2c.Instance = I2C1;
	this->hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	this->hi2c.Init.ClockSpeed = 400000;
	this->hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	this->hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	this->hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	this->hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	this->hi2c.Init.OwnAddress1 = 0;
	this->hi2c.Init.OwnAddress2 = 0;

	if (HAL_I2C_DeInit(&this->hi2c) != HAL_OK)
		return HAL_ERROR;

	return HAL_I2C_Init(&this->hi2c);
}*/

}
