/*
 * NEO_M8N.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#ifndef NEO_M8N_H_
#define NEO_M8N_H_

#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <string.h>

class NEO_M8N
{
private:
	uint32_t readPos;
	const uint8_t MAX_NULL_BYTES = 0;
	uint32_t size;
	char buffer[255];

	HAL_StatusTypeDef UART_Init();
	uint32_t findString(char * str);
	uint8_t readByte(char *data);
	void processMessage(char *msg, uint8_t len);
public:
	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_usart1_rx;
	uint8_t data[4096];
	uint32_t writePos;

	NEO_M8N();
	void Init();
	void ParseData();
};

#endif /* NEO_M8N_H_ */
