/*
 * Logger.h
 *
 *  Created on: 8. 2. 2017
 *      Author: michp
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "stm32f4xx_hal.h"
#include "string.h"

namespace flyhero {

class Logger {
private:
	Logger(){};
	Logger(Logger const&){};
	Logger& operator=(Logger const&){};

	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_usart2_tx;

public:
	static Logger& Instance();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Print(uint8_t *data, uint16_t len);

	DMA_HandleTypeDef* Get_DMA_Tx_Handle();
	UART_HandleTypeDef* Get_UART_Handle();
};

}

#endif /* LOGGER_H_ */
