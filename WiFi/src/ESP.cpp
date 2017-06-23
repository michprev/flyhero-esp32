/*
 * ESP.cpp
 *
 *  Created on: 1. 4. 2017
 *      Author: michp
 */

#include <ESP.h>
#include "ESP32.h"
#include "ESP8266.h"

namespace flyhero {

//#define LOG

extern "C" {
	void DMA1_Stream1_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(ESP::Instance()->Get_DMA_Rx_Handle());
	}

	void DMA1_Stream3_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(ESP::Instance()->Get_DMA_Tx_Handle());
	}

	void USART3_IRQHandler(void)
	{
		HAL_UART_IRQHandler(ESP::Instance()->Get_UART_Handle());
	}
}

ESP_Device ESP::device = NONE;

ESP* ESP::Create_Instance(ESP_Device dev) {
	switch (dev) {
	case ESP8266:
		ESP::device = ESP8266;
		return ESP8266::Instance();
	case ESP32:
		ESP::device = ESP32;
		return ESP32::Instance();
	}
}

ESP* ESP::Instance() {
	switch (ESP::device) {
	case ESP8266:
		return ESP8266::Instance();
	case ESP32:
		return ESP32::Instance();
	}
}

ESP::ESP() : connections{{this, '0'}, {this, '1'}, {this, '2'}, {this, '3'}, {this, '4'}} {

}

DMA_HandleTypeDef* ESP::Get_DMA_Tx_Handle() {
	return &this->hdma_usart3_tx;
}

DMA_HandleTypeDef* ESP::Get_DMA_Rx_Handle() {
	return &this->hdma_usart3_rx;
}

UART_HandleTypeDef* ESP::Get_UART_Handle() {
	return &this->huart;
}

ESP_State ESP::Get_State() {
	return this->state;
}

ESP_Connection* ESP::Get_Connection(uint8_t link_ID) {
	return &(this->connections[link_ID - '0']);
}

void ESP::reset() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

HAL_StatusTypeDef ESP::UART_Init(uint32_t baudrate)
{
	if (__USART3_IS_CLK_DISABLED())
		__USART3_CLK_ENABLE();
	if (__GPIOC_IS_CLK_DISABLED())
		__GPIOC_CLK_ENABLE();
	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	/**USART3 GPIO Configuration
	PC10     ------> USART3_TX
	PC11     ------> USART3_RX
	*/
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	this->huart.Instance = USART3;
	this->huart.Init.BaudRate = baudrate;
	this->huart.Init.WordLength = UART_WORDLENGTH_8B;
	this->huart.Init.StopBits = UART_STOPBITS_1;
	this->huart.Init.Parity = UART_PARITY_NONE;
	this->huart.Init.Mode = UART_MODE_TX_RX;
	this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	this->huart.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&this->huart))
		return HAL_ERROR;


	this->hdma_usart3_rx.Instance = DMA1_Stream1;
	this->hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
	this->hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	this->hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	this->hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	this->hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	this->hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	this->hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
	this->hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
	this->hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&this->hdma_usart3_rx) != HAL_OK)
	{
		return HAL_ERROR;
	}

	__HAL_LINKDMA(&this->huart, hdmarx, this->hdma_usart3_rx);

	this->hdma_usart3_tx.Instance = DMA1_Stream3;
	this->hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
	this->hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	this->hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	this->hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
	this->hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	this->hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	this->hdma_usart3_tx.Init.Mode = DMA_NORMAL;
	this->hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
	this->hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&this->hdma_usart3_tx) != HAL_OK)
	{
		return HAL_ERROR;
	}

	__HAL_LINKDMA(&this->huart, hdmatx, this->hdma_usart3_tx);

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	return HAL_OK;
}

uint32_t ESP::bytes_available() {
	uint32_t available_from_begin = this->BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&this->hdma_usart3_rx);

	if (available_from_begin >= this->readPos.pos)
		return available_from_begin - this->readPos.pos;
	else
		return this->BUFFER_SIZE - this->readPos.pos + available_from_begin;
}

void ESP::Process_Data() {
	if (this->bytes_available() > 0) {

		if (this->inIPD && this->IPD_received < this->IPD_size) {
			char c = this->buffer[this->readPos.pos];

			this->IPD_buffer[this->IPD_received] = c;

			this->IPD_received++;
			this->readPos.pos = this->readPos.add(1);

			if (this->IPD_received == this->IPD_size) {
				this->inIPD = false;

				if (this->IPD_callback != NULL)
					this->IPD_callback(this->link_ID, this->IPD_buffer, this->IPD_size);
			}
		}
		else {
			char c = this->buffer[this->readPos.pos];

			this->processing_buffer[this->processedLength] = c;

			/*if (!((c >= ' ' && c <= '~')
				|| c == '\r' || c == '\n')) {
					// TODO check is ASCII
					return;
				}*/

			this->processedLength++;
			this->readPos.pos = this->readPos.add(1);

			if (c == '\n') {
				this->parse((char*)this->processing_buffer, this->processedLength);

				this->processedLength = 0;
			}
			else if (c == '>') {

#ifdef LOG
				printf("[%d]>\n", HAL_GetTick() - this->timestamp);
#endif

				this->wait_for_wrap = false;

				this->processedLength = 0;
			}
			else if (c == ':' && strncmp("+IPD", (char*)this->processing_buffer, 4) == 0) {
				char lengthBuffer[10] = { '\0' };
				uint8_t lengthPos = 0;
				uint8_t commaCount = 0;

				for (uint8_t i = 4; i < this->processedLength; i++) {
					if (this->processing_buffer[i] == ',')
						commaCount++;
					else if (commaCount == 1)
						this->link_ID = this->processing_buffer[i];
					else if (commaCount == 2 && this->processing_buffer[i] != ':') {
						lengthBuffer[lengthPos] = this->processing_buffer[i];
						lengthPos++;
					}
				}

				this->inIPD = true;
				this->IPD_size = atoi(lengthBuffer);
				this->IPD_received = 0;

#ifdef LOG
				this->processing_buffer[this->processedLength] = '\0';
				printf("[%d]%s\n", HAL_GetTick() - this->timestamp, this->processing_buffer);
#endif

				this->processedLength = 0;
			}
			else if (this->processedLength >= this->MAX_PARSE_SIZE) {
				this->processedLength = 0;
			}
		}
	}
}

void ESP::parse(char *str, uint16_t length) {
#ifdef LOG
	if (strncmp("\r\n", str, 2) != 0 && strncmp("\n", str, 1) != 0) {
		if (length >= 9 && strncmp("CONNECT\r\n", str + length - 9, 9) == 0)
			this->timestamp = HAL_GetTick();

		printf("[%d]", HAL_GetTick() - this->timestamp);
		for (uint16_t i = 0; i < length; i++)
			printf("%c", *(str + i));
	}
#endif


	if (length >= 9 && strncmp("CONNECT\r\n", str + length - 9, 9) == 0) {
		this->connections[str[0] - '0'].Connected();

		return;
	}
	else if (length >= 8 && strncmp("CLOSED\r\n", str + length - 8, 8) == 0) {
		this->connections[str[0] - '0'].Closed();

		return;
	}
	else if (length >= 4 && strncmp("Recv", str, 4) == 0) {
		return;
	}
	else if (length >= 3 && strncmp("id:", str, 3) == 0) {
		return;
	}


	switch (length) {
	case 1:
		if (strncmp("\n", str, 1) == 0) {
			return;
		}

		break;
	case 2:
		if (strncmp("\r\n", str, length) == 0) {
			return;
		}


		break;
	case 3:
		if (strncmp(" \r\n", str, length) == 0) {
			return;
		}
		break;
	case 4:
		if (strncmp("OK\r\n", str, length) == 0)
			this->state = ESP_READY;
		return;
		break;
	case 6:
		if (strncmp("ATE0\r\n", str, length) == 0) {
			return;
		}

		break;
	case 7:
		if (strncmp("ready\r\n", str, length) == 0) {
			if (this->ready) {
				// TODO handle reset
			}
			else
				this->ready = true;

		}
		else if (strncmp("ERROR\r\n", str, length) == 0) {
			this->state = ESP_ERROR;
			return;
		}

		break;
	case 9:
		if (strncmp("SEND OK\r\n", str, length) == 0) {
			this->state = ESP_READY;
			return;
		}

		break;
	case 11:
		if (strncmp("SEND FAIL\r\n", str, length) == 0) {
			this->state = ESP_ERROR;
			return;
		}

		break;
	default:

		if (strncmp("+SYSRAM", str, 7) == 0) {

		}

		break;
	}


	if (this->ready) {
		for (uint16_t i = 0; i < length; i++)
			printf("%c", *(str + i));
	}
}

HAL_StatusTypeDef ESP::Send_Begin(const char *command) {
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, (uint8_t*)command, strlen(command));

#ifdef LOG
	printf("[%d]    %s\n", HAL_GetTick() - this->timestamp, command);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	return status;
}
HAL_StatusTypeDef ESP::Send_Begin(uint8_t *data, uint16_t count) {
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, data, count);

#ifdef LOG
	printf("[%d]    Sent %d bytes\n", HAL_GetTick() - this->timestamp, count);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	return status;
}

HAL_StatusTypeDef ESP::Send(const char *command)
{
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, (uint8_t*)command, strlen(command));

#ifdef LOG
	printf("[%d]    %s\n", HAL_GetTick() - this->timestamp, command);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	// TODO implement timeout
	while (this->state == ESP_SENDING) {
		this->Process_Data();
	}

	// TODO check state
	if (this->state != ESP_READY) {
		return HAL_ERROR;
	}

	return status;
}

HAL_StatusTypeDef ESP::Send(uint8_t *data, uint16_t count)
{
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, data, count);

#ifdef LOG
	printf("[%d]    Sent %d bytes\n", HAL_GetTick() - this->timestamp, count);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	// TODO implement timeout
	while (this->state == ESP_SENDING) {
		this->Process_Data();
	}

	// TODO check state

	if (this->state != ESP_READY) {
		return HAL_ERROR;
	}

	return status;
}

void ESP::Set_Wait_For_Wrap(bool value) {
	this->wait_for_wrap = value;
}

bool ESP::Get_Wait_For_Wrap() {
	return this->wait_for_wrap;
}

} /* namespace The_Eye */
