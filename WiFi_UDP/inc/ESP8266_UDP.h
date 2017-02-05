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
#include "LEDs.h"

enum ESP_State { ESP_SENDING, ESP_READY, ESP_AWAITING_BODY, ESP_ERROR };

class ESP8266_UDP
{
private:
	ESP8266_UDP();
	ESP8266_UDP(ESP8266_UDP const&){};
	ESP8266_UDP& operator=(ESP8266_UDP const&){};
	static ESP8266_UDP* pInstance;

	const static uint32_t SIZE = 4096;
	const uint8_t MAX_NULL_BYTES = 4;

	char clientIP[16] = { '\0' };
	uint16_t clientPort;
	uint8_t IPD_Data[1024];
	bool inIPD;
	uint32_t readPos;
	bool handshaken;

	HAL_StatusTypeDef UART_Init();
	HAL_StatusTypeDef UART_Send(uint8_t *data, uint16_t size);
	void TIM_Init();
	uint32_t getTick();
	int32_t findString(const char *str);
	uint8_t readByte(uint8_t *data, bool checkNull = false);
	HAL_StatusTypeDef send(const char *);
	HAL_StatusTypeDef waitReady(uint16_t delay = 5000);

public:
	ESP_State State;
	uint8_t data[SIZE] = { '\0' };
	DMA_HandleTypeDef hdma_usart3_rx;
	DMA_HandleTypeDef hdma_usart3_tx;
	UART_HandleTypeDef huart;
	TIM_HandleTypeDef htim5;
	bool Ready;
	bool Output;
	void(*IPD_Callback)(uint8_t *data, uint16_t length);

	static ESP8266_UDP* Instance();
	void ProcessData();
	HAL_StatusTypeDef SendUDP_Header(uint16_t length);
	HAL_StatusTypeDef SendUDP(uint8_t *data, uint16_t length);
	void Init();
	void Reset();
};

#endif /* ESP8266_UDP_H_ */
