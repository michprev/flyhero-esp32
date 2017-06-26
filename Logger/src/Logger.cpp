/*
 * Logger.cpp
 *
 *  Created on: 8. 2. 2017
 *      Author: michp
 */

#include "Logger.h"

namespace flyhero {

extern "C" {
	void DMA1_Stream6_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(Logger::Instance().Get_DMA_Tx_Handle());
	}

	void USART2_IRQHandler(void)
	{
		HAL_UART_IRQHandler(Logger::Instance().Get_UART_Handle());
	}
}

Logger& Logger::Instance() {
	static Logger instance;

	return instance;
}

HAL_StatusTypeDef Logger::Init() {
	if (__GPIOA_IS_CLK_DISABLED())
		__GPIOA_CLK_ENABLE();

	if (__USART2_IS_CLK_DISABLED())
		__USART2_CLK_ENABLE();

	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	// PA2 TX
	// PA3 RX
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin			 = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		 = GPIO_PULLUP;
	GPIO_InitStruct.Speed		 = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	 = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	this->hdma_usart2_tx.Instance					 = DMA1_Stream6;
	this->hdma_usart2_tx.Init.Channel				 = DMA_CHANNEL_4;
	this->hdma_usart2_tx.Init.Direction				 = DMA_MEMORY_TO_PERIPH;
	this->hdma_usart2_tx.Init.PeriphInc				 = DMA_PINC_DISABLE;
	this->hdma_usart2_tx.Init.MemInc				 = DMA_MINC_ENABLE;
	this->hdma_usart2_tx.Init.PeriphDataAlignment	 = DMA_PDATAALIGN_BYTE;
	this->hdma_usart2_tx.Init.MemDataAlignment		 = DMA_MDATAALIGN_BYTE;
	this->hdma_usart2_tx.Init.Mode					 = DMA_NORMAL;
	this->hdma_usart2_tx.Init.Priority				 = DMA_PRIORITY_LOW;
	this->hdma_usart2_tx.Init.FIFOMode				 = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&this->hdma_usart2_tx))
		return HAL_ERROR;

	__HAL_LINKDMA(&this->huart, hdmatx, this->hdma_usart2_tx);


	this->huart.Instance = USART2;
	this->huart.Init.BaudRate = 2000000;
	this->huart.Init.WordLength = UART_WORDLENGTH_8B;
	this->huart.Init.StopBits = UART_STOPBITS_1;
	this->huart.Init.Parity = UART_PARITY_NONE;
	this->huart.Init.Mode = UART_MODE_TX_RX;
	this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	this->huart.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&this->huart))
		return HAL_ERROR;

	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	return HAL_OK;
}

HAL_StatusTypeDef Logger::Print(uint8_t *data, uint16_t len) {
	return HAL_UART_Transmit_DMA(&this->huart, data, len);
}

DMA_HandleTypeDef* Logger::Get_DMA_Tx_Handle() {
	return &this->hdma_usart2_tx;
}

UART_HandleTypeDef* Logger::Get_UART_Handle() {
	return &this->huart;
}

}
