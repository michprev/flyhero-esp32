/*
 * ESP32.cpp
 *
 *  Created on: 10. 3. 2017
 *      Author: michp
 */

#include <ESP32.h>

//#define LOG

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


	memset(this->buffer, 0, this->BUFFER_SIZE);
	memset(this->processing_buffer, 0, this->MAX_PARSE_SIZE);

	if (UART_Init() != HAL_OK) {
		//LEDs::TurnOn(LEDs::Green | LEDs::Orange | LEDs::Yellow);
		while (true);
	}
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

HAL_StatusTypeDef ESP32::Init() {
	this->Send("AT+RST\r\n");

	while (!this->ready) {
		this->Process_Data();
	}

	this->Send("ATE0\r\n");
	//this->send("AT+SYSRAM?\r\n");
	this->Send("AT+CWMODE=2\r\n");
	this->Send("AT+CWSAP=\"DRON_WIFI\",\"123456789\",5,3,1,1\r\n");
	this->Send("AT+CWDHCP=1,1\r\n");
	this->Send("AT+CIPMUX=1\r\n");
	this->Send("AT+CIPSERVER=1,80\r\n");
}

HAL_StatusTypeDef ESP32::UART_Init()
{
	if (__GPIOC_IS_CLK_DISABLED())
		__GPIOC_CLK_ENABLE();

	if (__USART3_IS_CLK_DISABLED())
		__USART3_CLK_ENABLE();

	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	// PC10 TX
	// PC11 RX

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

	__HAL_DMA_RESET_HANDLE_STATE(&this->hdma_usart3_rx);

	if (HAL_DMA_DeInit(&this->hdma_usart3_rx))
		return HAL_ERROR;

	if (HAL_DMA_Init(&this->hdma_usart3_rx))
		return HAL_ERROR;

	__HAL_LINKDMA(&huart, hdmarx, this->hdma_usart3_rx);

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

	__HAL_DMA_RESET_HANDLE_STATE(&this->hdma_usart3_tx);

	if (HAL_DMA_DeInit(&this->hdma_usart3_tx))
		return HAL_ERROR;

	if (HAL_DMA_Init(&this->hdma_usart3_tx))
		return HAL_ERROR;

	__HAL_LINKDMA(&huart, hdmatx, this->hdma_usart3_tx);


	this->huart.Instance = USART3;
	this->huart.Init.BaudRate = 115200;
	this->huart.Init.WordLength = UART_WORDLENGTH_8B;
	this->huart.Init.StopBits = UART_STOPBITS_1;
	this->huart.Init.Parity = UART_PARITY_NONE;
	this->huart.Init.Mode = UART_MODE_TX_RX;
	this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	this->huart.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_DeInit(&this->huart) || HAL_UART_Init(&this->huart))
		return HAL_ERROR;

	if (HAL_UART_Receive_DMA(&this->huart, this->buffer, this->BUFFER_SIZE))
		return HAL_ERROR;

	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	return HAL_OK;
}

bool ESP32::next_bytes_null() {
	for (uint8_t i = 0; i < this->MAX_NULL_BYTES; i++) {
		if (this->buffer[this->readPos.add(i)] != '\0')
			return false;
	}

	return true;
}

void ESP32::Process_Data() {
	while (!this->next_bytes_null() &&
			this->buffer[this->readPos.previous()] == '\0') {

		if (this->inIPD && this->IPD_received < this->IPD_size) {
			char c = this->buffer[this->readPos.pos];
			this->buffer[this->readPos.pos] = '\0';

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
			this->buffer[this->readPos.pos] = '\0';

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
				printf(">\n");
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
				printf("%s\n", this->processing_buffer);
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
		for (uint16_t i = 0; i < length; i++)
			printf("%c", *(str + i));
	}
#endif


	if (length >= 9 && strncmp("CONNECT\r\n", str + length - 9, 9) == 0) {
		this->TCP_connections[str[0] - '0'].Connected();

		// TODO double-digit LinkID?

		return;
	}
	else if (length >= 8 && strncmp("CLOSED\r\n", str + length - 8, 8) == 0) {
		this->TCP_connections[str[0] - '0'].Closed();

		// TODO double-digit LinkID?

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
		/*else if (this->ready) {
			for (uint16_t i = 0; i < length; i++)
				printf("%c", *(str + i));
		}*/

		break;
	}


	if (this->ready) {
		for (uint16_t i = 0; i < length; i++)
			printf("%c", *(str + i));
	}
}

HAL_StatusTypeDef ESP32::Send(const char *command)
{
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit(&this->huart, (uint8_t*)command, strlen(command), this->UART_TIMEOUT);

#ifdef LOG
	printf("    %s\n", command);
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

HAL_StatusTypeDef ESP32::Send(const char * data, uint16_t count)
{
	this->state = ESP_SENDING;

	HAL_StatusTypeDef status = HAL_UART_Transmit(&this->huart, (uint8_t*)data, count, 1000);

#ifdef LOG
	printf("    Sent %d bytes\n", count);
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

HAL_StatusTypeDef ESP32::Send_File(uint8_t link_ID, const char *header, const char *body, uint16_t bodySize)
{
	return this->TCP_connections[link_ID - '0'].Send_File(this->send_buffer, link_ID, header, body, bodySize);
}

void ESP32::Set_Wait_For_Wrap(bool value) {
	this->wait_for_wrap = value;
}

bool ESP32::Get_Wait_For_Wrap() {
	return this->wait_for_wrap;
}

