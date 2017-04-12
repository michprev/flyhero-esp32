/*
 * ArduCAM.h
 *
 *  Created on: 9. 3. 2017
 *      Author: michp
 */

#ifndef ARDUCAM_H_
#define ARDUCAM_H_

#include "stm32f4xx_hal.h"
#include "ov5640_regs.h"

class ArduCAM {
private:
	ArduCAM(){};
	ArduCAM(ArduCAM const&){};
	ArduCAM& operator=(ArduCAM const&){};
	static ArduCAM* pInstance;
	I2C_HandleTypeDef hi2c;


	static const uint8_t I2C_ADDRESS = 0x78;
	static const uint16_t I2C_TIMEOUT = 1000;
	static const uint16_t SPI_TIMEOUT = 1000;
	static const uint32_t MAX_FIFO_SIZE = 0x7FFFFF;

	HAL_StatusTypeDef i2c_read(uint16_t reg_address, uint8_t *data);
	HAL_StatusTypeDef i2c_write(uint16_t reg_address, uint8_t *data);
	HAL_StatusTypeDef i2c_write_regs(const sensor_reg *reg_list);
	HAL_StatusTypeDef spi_read(uint8_t reg_address, uint8_t *data);
	HAL_StatusTypeDef spi_write(uint8_t reg_address, uint8_t *data);

	HAL_StatusTypeDef read_fifo_burst();

	void DMA_init();
	void DMA_deinit();

public:
	SPI_HandleTypeDef hspi;
	DMA_HandleTypeDef hdma_spi1_rx;
	DMA_HandleTypeDef hdma_spi1_tx;
	uint8_t buffer[20000] = { '\0' };
	uint32_t image_size = 0;

	static ArduCAM* Instance();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Capture();
};

#endif /* ARDUCAM_H_ */
