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
	ESP8266_UDP();
	ESP8266_UDP(ESP8266_UDP const&){};
	ESP8266_UDP& operator=(ESP8266_UDP const&){};
	static ESP8266_UDP* pInstance;

	const static uint32_t size = 2048;
	uint8_t MAX_NULL_BYTES = 5;
	char expectedResponse[20] = { '\0' };
	char clientIP[16] = { '\0' };
	uint16_t clientPort;
	uint8_t IPD_Data[1024];
	char sendBuffer[2048] = { '\0' };
	WaitFlag waitFlag;
	bool inIPD;

	HAL_StatusTypeDef UART_Init();
	int32_t findString(const char *str);
	uint8_t readByte(uint8_t *data, bool checkNull = false);
	void processData();
	HAL_StatusTypeDef send(const char *);

public:
	uint8_t data[size] = { '\0' };
	DMA_HandleTypeDef hdma_usart3_rx;
	UART_HandleTypeDef huart;
	bool ready;
	bool handshaken;
	bool output;
	uint32_t readPos;
	void(*IPD_Callback)(uint8_t *data, uint16_t length);

	static ESP8266_UDP* Instance();
	HAL_StatusTypeDef SendUDP(uint8_t *data, uint16_t length);
	HAL_StatusTypeDef WaitReady(uint16_t delay = 5000);
	void Init();
	void Reset();
};

#endif /* ESP8266_UDP_H_ */
