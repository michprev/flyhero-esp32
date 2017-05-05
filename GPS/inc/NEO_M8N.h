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

namespace flyhero {

struct GPS_Data {
	double latitude;
	double longitude;
	char NS;
	char EW;
};

class NEO_M8N
{
private:
	NEO_M8N();
	NEO_M8N(NEO_M8N const&){};
	NEO_M8N& operator=(NEO_M8N const&){};
	static NEO_M8N* pInstance;

	UART_HandleTypeDef huart;
	uint32_t writePos;
	uint32_t readPos;
	const uint8_t MAX_NULL_BYTES = 0;
	uint32_t size;
	char buffer[255];
	uint8_t data[4096] = { '\0' };

	HAL_StatusTypeDef UART_Init();
	uint32_t findString(char * str);
	uint8_t readByte(char *data);
	void processMessage(char *msg, uint8_t len);
public:
	DMA_HandleTypeDef hdma_usart1_rx;
	GPS_Data Data;
	double UTC_Time = 0;

	static NEO_M8N* Instance();
	void Init();
	void ParseData();
};

}

#endif /* NEO_M8N_H_ */
