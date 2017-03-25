/*
 * ESP32.cpp
 *
 *  Created on: 10. 3. 2017
 *      Author: michp
 */

#include <ESP32.h>

//#define LOG
#define ESP8266

ESP32* ESP32::pInstance = NULL;

ESP32* ESP32::Instance() {
	if (ESP32::pInstance == NULL)
		pInstance = new ESP32();

	return pInstance;
}

ESP32::ESP32() {
	this->readPos.pos = 0;
	this->processedLength = 0;
	this->ready = false;
	this->inIPD = false;
	this->IPD_Callback = NULL;
	this->state = ESP_READY;
	this->timestamp = HAL_GetTick();
	this->link_ID = -1;
	this->wait_for_wrap = false;
	this->IPD_received = 0;
	this->IPD_size = 0;
	this->huart = UART_HandleTypeDef();
	this->hdma_usart3_rx = DMA_HandleTypeDef();
	this->hdma_usart3_tx = DMA_HandleTypeDef();
}

DMA_HandleTypeDef* ESP32::Get_DMA_Tx_Handle() {
	return &this->hdma_usart3_tx;
}

DMA_HandleTypeDef* ESP32::Get_DMA_Rx_Handle() {
	return &this->hdma_usart3_rx;
}

UART_HandleTypeDef* ESP32::Get_UART_Handle() {
	return &this->huart;
}

HTTP_Server* ESP32::Get_HTTP_Server() {
	return &this->HTTP_server;
}

ESP_State ESP32::Get_State() {
	return this->state;
}

TCP_Connection* ESP32::Get_TCP_Connection(uint8_t link_ID) {
	return &(this->TCP_connections[link_ID - '0']);
}

void ESP32::reset() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

HAL_StatusTypeDef ESP32::Init() {
	if (this->UART_Init(115200) != HAL_OK) {
		//LEDs::TurnOn(LEDs::Green | LEDs::Orange | LEDs::Yellow);
		while (true);
	}

	// PB7 RST

	GPIO_InitTypeDef rst;
	rst.Pin = GPIO_PIN_7;
	rst.Mode = GPIO_MODE_OUTPUT_PP;
	rst.Pull = GPIO_PULLUP;;
	rst.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &rst);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	this->reset();

	HAL_Delay(1000);
	HAL_UART_Receive_DMA(&this->huart, this->buffer, this->BUFFER_SIZE);


#ifdef ESP8266
	this->Send("AT+UART_CUR=4500000,8,1,0,0\r\n");
	HAL_DMA_Abort(&this->hdma_usart3_rx);
	HAL_DMA_Abort(&this->hdma_usart3_tx);
	HAL_UART_DeInit(&this->huart);
	this->UART_Init(4500000);
	HAL_Delay(1000);
	HAL_UART_Receive_DMA(&this->huart, this->buffer, this->BUFFER_SIZE);
#else
	this->Send("AT+UART_CUR=5000000,8,1,0,0\r\n");
	HAL_DMA_Abort(&this->hdma_usart3_rx);
	HAL_DMA_Abort(&this->hdma_usart3_tx);
	HAL_UART_DeInit(&this->huart);
	this->UART_Init(5000000);
	HAL_Delay(1000);
	HAL_UART_Receive_DMA(&this->huart, this->buffer, this->BUFFER_SIZE);
#endif


	this->Send("ATE0\r\n");
	//this->send("AT+SYSRAM?\r\n");
	this->Send("AT+CWMODE=2\r\n");
	this->Send("AT+CWSAP=\"DRON_WIFI\",\"123456789\",5,3,1,1\r\n");
#ifdef ESP8266
	this->Send("AT+CWDHCP=0,1\r\n");
#else
	this->Send("AT+CWDHCP=1,1\r\n");
#endif
	this->Send("AT+CIPMUX=1\r\n");
	this->Send("AT+CIPSERVER=1,80\r\n");
	this->Send("AT+CIPSTART=4,\"UDP\",\"0\",0,4789,1\r\n");
}

HAL_StatusTypeDef ESP32::UART_Init(uint32_t baudrate)
{
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

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

	this->readPos.pos = 0;

	return HAL_OK;
}

uint32_t ESP32::bytes_available() {
	uint32_t available_from_begin = this->BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&this->hdma_usart3_rx);

	if (available_from_begin >= this->readPos.pos)
		return available_from_begin - this->readPos.pos;
	else
		return this->BUFFER_SIZE - this->readPos.pos + available_from_begin;
}

void ESP32::Process_Data() {
	while (this->bytes_available() > 0) {

		if (this->inIPD && this->IPD_received < this->IPD_size) {
			char c = this->buffer[this->readPos.pos];

			this->IPD_buffer[this->IPD_received] = c;

			this->IPD_received++;
			this->readPos.pos = this->readPos.add(1);

			if (this->IPD_received == this->IPD_size) {
				this->inIPD = false;

				if (this->IPD_Callback != NULL)
					this->IPD_Callback(this->link_ID, this->IPD_buffer, this->IPD_size);
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
				if (this->processedLength > 1)
					printf("WTF\n");

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

void ESP32::parse(char *str, uint16_t length) {
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
		this->TCP_connections[str[0] - '0'].Connected();

		return;
	}
	else if (length >= 8 && strncmp("CLOSED\r\n", str + length - 8, 8) == 0) {
		this->TCP_connections[str[0] - '0'].Closed();

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

HAL_StatusTypeDef ESP32::Send_Begin(const char *command) {
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, (uint8_t*)command, strlen(command));

#ifdef LOG
	printf("[%d]    %s\n", HAL_GetTick() - this->timestamp, command);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	return status;
}
HAL_StatusTypeDef ESP32::Send_Begin(uint8_t *data, uint16_t count) {
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&this->huart, data, count);

#ifdef LOG
	printf("[%d]    Sent %d bytes\n", HAL_GetTick() - this->timestamp, count);
#endif

	if (status != HAL_OK)
		printf("UART error\n");

	return status;
}

HAL_StatusTypeDef ESP32::Send(const char *command)
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

HAL_StatusTypeDef ESP32::Send(uint8_t *data, uint16_t count)
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

void ESP32::Set_Wait_For_Wrap(bool value) {
	this->wait_for_wrap = value;
}

bool ESP32::Get_Wait_For_Wrap() {
	return this->wait_for_wrap;
}

