/*
 * ArduCAM.cpp
 *
 *  Created on: 9. 3. 2017
 *      Author: michp
 */

#include <ArduCAM.h>

ArduCAM* ArduCAM::pInstance = NULL;

ArduCAM* ArduCAM::Instance() {
	if (ArduCAM::pInstance == NULL)
		pInstance = new ArduCAM();

	return pInstance;
}

HAL_StatusTypeDef ArduCAM::Init() {
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	if (__GPIOA_IS_CLK_DISABLED())
		__GPIOA_CLK_ENABLE();

	if (__SPI1_IS_CLK_DISABLED())
		__SPI1_CLK_ENABLE();

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
	if (HAL_I2C_Init(&this->hi2c) != HAL_OK)
		return HAL_ERROR;

	GPIO_InitTypeDef NSS;
	NSS.Pin = GPIO_PIN_6;
	NSS.Mode = GPIO_MODE_OUTPUT_PP;
	NSS.Speed = GPIO_SPEED_HIGH;
	NSS.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &NSS);

	// disable slave
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	this->hspi.Instance = SPI1;
	this->hspi.Init.Mode = SPI_MODE_MASTER;
	this->hspi.Init.Direction = SPI_DIRECTION_2LINES;
	this->hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	this->hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	this->hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	this->hspi.Init.NSS = SPI_NSS_SOFT;
	this->hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	this->hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	this->hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	this->hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	this->hspi.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&this->hspi) != HAL_OK)
		return HAL_ERROR;

	uint8_t data = 0x55;
	uint8_t response;

	uint32_t ticks = HAL_GetTick();

	uint8_t vid, pid;
	this->i2c_read(OV5640_CHIPID_HIGH, &vid);
	this->i2c_read(OV5640_CHIPID_LOW, &pid);

	do {
		this->spi_write(0x00, &data);
		this->spi_read(0x00, &response);
	} while (response != 0x55 && HAL_GetTick() - ticks < 5000);

	if (response != 0x55)
		return HAL_TIMEOUT;



	data = 0x11;
	this->i2c_write(0x3103, &data);

	data = 0x82;
	this->i2c_write(0x3008, &data);

	HAL_Delay(100);

	this->i2c_write_regs(OV5640YUV_Sensor_Dvp_Init);

	HAL_Delay(500);

	this->i2c_write_regs(OV5640_JPEG_QSXGA);
	this->i2c_write_regs(OV5640_QSXGA2QVGA);
	data = 0x04;
	this->i2c_write(0x4407, &data);

	this->spi_read(0x03, &data);
	data |= 0x02;
	this->spi_write(0x03, &data);

	data = 0x01;
	this->spi_write(0x04, &data);

	data = 0x00;
	this->spi_write(0x01, &data);

	return HAL_OK;
}

HAL_StatusTypeDef ArduCAM::Capture() {
	uint8_t data;

	uint32_t gg = HAL_GetTick();

	data = 0x01;
	this->spi_write(0x04, &data);

	data = 0x02;
	this->spi_write(0x04, &data);

	do {
		this->spi_read(0x41, &data);

		// data ready
		if (data & 0x08) {
			this->read_fifo_burst();

			data = 0x01;
			this->spi_write(0x04, &data);

			break;
		}
	} while (true);

	return HAL_OK;
}

HAL_StatusTypeDef ArduCAM::read_fifo_burst() {
	uint32_t fifo_size = 0;

	uint8_t data;

	this->spi_read(0x42, &data);
	fifo_size |= data;

	this->spi_read(0x43, &data);
	fifo_size |= (data << 8);

	this->spi_read(0x44, &data);
	fifo_size |= ((data & 0x7F) << 16);

	if (fifo_size == 0 || fifo_size >= 20000) {
		data = 0x01;
		this->spi_write(0x04, &data);

		return HAL_ERROR;
	}

	this->image_size = fifo_size;

	// enable fifo read burst mode
	data = 0x3C;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&this->hspi,& data, 1, this->SPI_TIMEOUT);

	uint32_t buffer_pos = 0;
	bool eoi = false;

	for (uint32_t i = 0; i < fifo_size; i++) {
		HAL_SPI_Receive(&this->hspi, this->buffer + buffer_pos, 1, this->SPI_TIMEOUT);

		if (!eoi)
			buffer_pos++;

		if (buffer_pos == 2) {
			if (this->buffer[0] == 0xFF && this->buffer[1] == 0xD8) {

			}
			else
				buffer_pos = 0;
		}
		else if (buffer_pos >= 4) {
			if (this->buffer[buffer_pos - 2] == 0xFF && this->buffer[buffer_pos - 1] == 0xD9)
				eoi = true;
		}
	}

	this->image_size = buffer_pos;

	data = 0x01;
	this->spi_write(0x04, &data);

	return HAL_OK;
}

HAL_StatusTypeDef ArduCAM::i2c_read(uint16_t reg_address, uint8_t *data) {
	return HAL_I2C_Mem_Read(&this->hi2c, this->I2C_ADDRESS, reg_address, I2C_MEMADD_SIZE_16BIT, data, 1, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef ArduCAM::i2c_write(uint16_t reg_address, uint8_t *data) {
	return HAL_I2C_Mem_Write(&this->hi2c, this->I2C_ADDRESS, reg_address, I2C_MEMADD_SIZE_16BIT, data, 1, this->I2C_TIMEOUT);
}

HAL_StatusTypeDef ArduCAM::i2c_write_regs(const sensor_reg *reg_list) {
	const sensor_reg *next = reg_list;
	uint8_t val;

	while ((next->reg != 0xFFFF) || (next->val != 0xFF)) {
		val = next->val;
		if (this->i2c_write(next->reg, &val) != HAL_OK)
			return HAL_ERROR;

		next++;
	}

	return HAL_OK;
}

HAL_StatusTypeDef ArduCAM::spi_read(uint8_t reg_address, uint8_t *data) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	reg_address &= 0x7F;

	HAL_SPI_Transmit(&this->hspi, &reg_address, 1, this->SPI_TIMEOUT);

	HAL_SPI_Receive(&this->hspi, data, 1, this->SPI_TIMEOUT);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

HAL_StatusTypeDef ArduCAM::spi_write(uint8_t reg_address, uint8_t *data) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	reg_address |= 0x80;

	HAL_SPI_Transmit(&this->hspi, &reg_address, 1, this->SPI_TIMEOUT);

	HAL_SPI_Transmit(&this->hspi, data, 1, this->SPI_TIMEOUT);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}