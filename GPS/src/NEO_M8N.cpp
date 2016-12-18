/*
 * NEO_M8N.cpp
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#include "NEO_M8N.h"

HAL_StatusTypeDef NEO_M8N::UART_Init()
{
	if (__GPIOA_IS_CLK_DISABLED())
		__GPIOA_CLK_ENABLE();

	if (__USART1_IS_CLK_DISABLED())
		__USART1_CLK_ENABLE();

	if (__DMA2_IS_CLK_DISABLED())
		__DMA2_CLK_ENABLE();

	/*USART1 GPIO Configuration
	PA9      ------> USART1_TX
	PA10     ------> USART1_RX
	*/
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	this->hdma_usart1_rx.Instance = DMA2_Stream2;
	this->hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	this->hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	this->hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	this->hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	this->hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	this->hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	this->hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	this->hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	this->hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	HAL_DMA_Init(&this->hdma_usart1_rx);

	this->huart.Instance = USART1;
	this->huart.Init.BaudRate = 9600;
	this->huart.Init.WordLength = UART_WORDLENGTH_8B;
	this->huart.Init.StopBits = UART_STOPBITS_1;
	this->huart.Init.Parity = UART_PARITY_NONE;
	this->huart.Init.Mode = UART_MODE_TX_RX;
	this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	this->huart.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&this->huart);

	__HAL_LINKDMA(&huart, hdmarx, this->hdma_usart1_rx);

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

NEO_M8N::NEO_M8N()
{
	this->readPos = 0;
	this->writePos = 0;
	this->size = 4096;
}

void NEO_M8N::Init()
{
	UART_Init();
	HAL_UART_Receive_DMA(&this->huart, this->data, 4096);
}

uint32_t NEO_M8N::findString(char * str)
{
	uint32_t length = strlen(str);
	uint32_t retval = 0;
	uint32_t read = this->readPos;
	bool stop = false;

	do {
		bool found = true;
		uint32_t tmpRead = read;

		if (read >= this->size)
			read = 0;

		for (uint32_t i = 0; i < length; i++) {
			if (tmpRead >= this->size)
				tmpRead = 0;

			if (this->data[tmpRead] == '\0') {
				bool skip = false;
				uint32_t nullCheckPos = tmpRead;
				for (uint8_t j = 0; j < this->MAX_NULL_BYTES - 1; j++) {
					if (nullCheckPos >= this->size)
						nullCheckPos = 0;

					if (this->data[nullCheckPos] != '\0') {
						skip = true;
						break;
					}
					nullCheckPos++;
				}

				if (!skip) {
					stop = true;
					found = false;
					break;
				}
			}

			if (this->data[tmpRead] != str[i]) {
				found = false;
				break;
			}
			tmpRead++;
		}

		if (found)
			return retval;

		read++;
		retval++;
	} while (!stop);

	return -1;
}

uint8_t NEO_M8N::readByte(char *data)
{
	if (this->data[readPos] == '\0') {
		return 0;
	}
	else if (data != NULL)
		*data = this->data[this->readPos];

	this->data[this->readPos] = '\0';
	this->readPos++;

	if (this->readPos == this->size)
		this->readPos = 0;

	return 1;
}

void NEO_M8N::processMessage(char *msg, uint8_t len)
{
	if (len < 6)
		return;

	// check checksum
	if (msg[len - 5] != '*')
		return;

	uint8_t sum = 0;

	if (msg[len - 3] >= '0' && msg[len - 3] <= '9')
		sum += msg[len - 3] - '0';
	else if (msg[len - 3] >= 'A' && msg[len - 3] <= 'F')
		sum += msg[len - 3] - 'A' + 10;
	else if (msg[len - 3] >= 'a' && msg[len - 3] <= 'f')
		sum += msg[len - 3] - 'a' + 10;

	if (msg[len - 4] >= '0' && msg[len - 4] <= '9')
		sum += (msg[len - 4] - '0') * 16;
	else if (msg[len - 4] >= 'A' && msg[len - 4] <= 'F')
		sum += (msg[len - 4] - 'A' + 10) * 16;
	else if (msg[len - 4] >= 'a' && msg[len - 4] <= 'f')
		sum += (msg[len - 4] - 'a' + 10) * 16;

	for (uint8_t i = 1; i < len - 5; i++)
		sum ^= msg[i];

	if (sum != 0)
		return;

	if (msg[0] != '$')
		return;

	if (strncmp("GP", msg + 1, 2) != 0 && strncmp("GL", msg + 1, 2) != 0 && strncmp("GA", msg + 1, 2) != 0 &&
		strncmp("GB", msg + 1, 2) != 0 && strncmp("GN", msg + 1, 2) != 0)
		return;

	// Datum Reference - NOT USED
	/*if (strncmp("DTM", msg + 3, 3) == 0) {
	}*/
	// GNSS Satellite Fault Detection
	if (strncmp("GBS", msg + 3, 3) == 0) {
		//printf("fhu");
	}
	// Global positioning system fix data
	else if (strncmp("GGA", msg + 3, 3) == 0) {

	}
	// Latitude and longitude, with time of position fix and status
	else if (strncmp("GLL", msg + 3, 3) == 0) {
		GPS_Data d;
		bool valid;
		uint8_t part = 0;
		uint8_t buffer[20];
		uint8_t bufferPos = 0;

		for (uint8_t i = 7; i <= len - 4; i++) {
			if (msg[i] == ',') {
				buffer[bufferPos] = '\0';

				switch (part) {
				case 0:
					if (bufferPos != 0)
						d.latitude = atof((char*)buffer);
					else
						d.latitude = 0;
					break;
				case 1:
					if (bufferPos != 0)
						d.NS = msg[i - 1];
					else
						d.NS = '\0';
					break;
				case 2:
					if (bufferPos != 0)
						d.longitude = atof((char*)buffer);
					else
						d.longitude = 0;
					break;
				case 3:
					if (bufferPos != 0)
						d.EW = msg[i - 1];
					else
						d.EW = '\0';
					break;
				case 4:
					if (bufferPos != 0)
						this->UTC_Time = atof((char*)buffer);
					break;
				case 5:
					if (bufferPos != 0)
						valid = (msg[i - 1] == 'A');
					else
						valid = false;
					break;
				}

				bufferPos = 0;
				part++;
			}
			else {
				buffer[bufferPos] = msg[i];
				bufferPos++;
			}
		}

		if (valid) {
			this->Data = d;
			printf("%f%c %f%c\n", d.latitude, d.NS, d.longitude, d.EW);
		}
	}
	// GNSS fix data
	else if (strncmp("GNS", msg + 3, 3) == 0) {

	}
	// GNSS Range Residuals
	else if (strncmp("GRS", msg + 3, 3) == 0) {

	}
	// GNSS DOP and Active Satellites
	else if (strncmp("GSA", msg + 3, 3) == 0) {

	}
	// GNSS Pseudo Range Error Statistics
	else if (strncmp("GST", msg + 3, 3) == 0) {

	}
	// GNSS Satellites in View
	else if (strncmp("GSV", msg + 3, 3) == 0) {

	}
	// Recommended Minimum data
	else if (strncmp("RMC", msg + 3, 3) == 0) {

	}
	// Text Transmission
	else if (strncmp("TXT", msg + 3, 3) == 0) {

	}
	// Dual ground/water distance
	else if (strncmp("VLW", msg + 3, 3) == 0) {

	}
	// Course over ground and Ground speed
	else if (strncmp("VTG", msg + 3, 3) == 0) {

	}
	// Time and Date - NOT USED
	/*else if (strncmp("ZDA", msg + 3, 3) == 0) {
		printf("date\n");
	}*/
	else
		printf("Unsupported message\n");
}

void NEO_M8N::ParseData()
{
	while (findString("\r\n") != -1) {
		uint8_t i = 0;
		uint8_t count;
		char c;

		do {
			count = readByte(&c);

			if (count == 1) {
				buffer[i] = c;
				i++;
			}
		} while (count == 1 && c != '\n');

		buffer[i] = '\0';

		processMessage(buffer, i);
	}
}
