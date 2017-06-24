/*
 * ArduCAMOV5642.h
 *
 *  Created on: 9. 4. 2017
 *      Author: michp
 */

#ifndef ARDUCAMO_V5642_H_
#define ARDUCAMOV_5642_H_

#include "ov5642_regs.h"
#include <stm32f4xx_hal.h>
#include "sensor_reg.h"

namespace flyhero {

class ArduCAM_OV5642 {
private:
	ArduCAM_OV5642(){};
	ArduCAM_OV5642(ArduCAM_OV5642 const&){};
	ArduCAM_OV5642& operator=(ArduCAM_OV5642 const&){};

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
	void delay_us(uint32_t us);

	void DMA_init();
	void DMA_deinit();
public:
	TIM_HandleTypeDef htim5;
	SPI_HandleTypeDef hspi;
	DMA_HandleTypeDef hdma_spi1_rx;
	DMA_HandleTypeDef hdma_spi1_tx;
	uint8_t buffer[20000] = { '\0' };
	uint32_t image_size = 0;

	static ArduCAM_OV5642& Instance();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Capture();
};

}

#endif /* ARDUCAMOV5642_H_ */
