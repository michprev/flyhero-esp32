/*
 * ESP8266_UDP.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#ifndef ESP8266_UDP_H_
#define ESP8266_UDP_H_

#include <stdlib.h>
#include <stdint.h>
#include <cstring>
#include <stm32f4xx_hal.h>

enum WaitFlag { WAIT_AT, WAIT_OK, WAIT_ERROR };

class ESP8266_UDP
{
private:
	uint8_t MAX_NULL_BYTES = 5;
	char expectedResponse[20] = { '\0' };
	char clientIP[16] = { '\0' };
	uint16_t clientPort;
	uint8_t IPD_Data[1024];
	char sendBuffer[2048] = { '\0' };
	WaitFlag waitFlag;
	bool inIPD;
	uint32_t readPos;
	uint32_t size;
	uint8_t *data;

	HAL_StatusTypeDef UART_Init();
	uint32_t findString(char *str);
	uint8_t readByte(uint8_t *data, bool checkNull = false);
	void processData();
	HAL_StatusTypeDef send(char *);

public:
	DMA_HandleTypeDef hdma_usart3_rx;
	UART_HandleTypeDef huart;
	bool ready;
	bool handshaken;
	bool output;
	void(*IPD_Callback)(uint8_t *data, uint16_t length);

	ESP8266_UDP(uint32_t size);
	HAL_StatusTypeDef SendUDP(uint8_t *data, uint16_t length);
	HAL_StatusTypeDef WaitReady(uint16_t delay = 5000);
	void Init();
};

#endif /* ESP8266_UDP_H_ */
