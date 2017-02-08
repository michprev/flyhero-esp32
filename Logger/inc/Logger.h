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

class Logger {
private:
	Logger(){};
	Logger(Logger const&){};
	Logger& operator=(Logger const&){};
	static Logger* pInstance;

public:
	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_uart5_tx;

	static Logger* Instance();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Print(char *str);
};

#endif /* LOGGER_H_ */
