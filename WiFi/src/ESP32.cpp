/*
 * ESP32.cpp
 *
 *  Created on: 10. 3. 2017
 *      Author: michp
 */

#include <ESP32.h>

namespace flyhero {

ESP32* ESP32::pInstance = NULL;

ESP* ESP32::Instance() {
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

	this->Send("AT+UART_CUR=5000000,8,1,0,0\r\n");
	HAL_DMA_Abort(&this->hdma_usart3_rx);
	HAL_DMA_Abort(&this->hdma_usart3_tx);
	HAL_UART_DeInit(&this->huart);
	this->UART_Init(5000000);
	HAL_Delay(1000);
	HAL_UART_Receive_DMA(&this->huart, this->buffer, this->BUFFER_SIZE);


	this->Send("ATE0\r\n");
	//this->send("AT+SYSRAM?\r\n");
	this->Send("AT+CWMODE=2\r\n");
	this->Send("AT+CWSAP=\"DRON_WIFI\",\"123456789\",5,3,1,1\r\n");
	this->Send("AT+CWDHCP=1,1\r\n");
	this->Send("AT+CIPMUX=1\r\n");
	this->Send("AT+CIPSERVER=1,80\r\n");
	this->Send("AT+CIPSTART=4,\"UDP\",\"0\",0,4789,1\r\n");
}

}
