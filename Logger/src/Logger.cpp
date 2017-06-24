/*
 * Logger.cpp
 *
 *  Created on: 8. 2. 2017
 *      Author: michp
 */

#include "Logger.h"

namespace flyhero {

extern "C" {
	void DMA1_Stream7_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(&Logger::Instance().hdma_uart5_tx);
	}

	void UART5_IRQHandler(void)
	{
		HAL_UART_IRQHandler(&Logger::Instance().huart);
	}
}

Logger& Logger::Instance() {
	static Logger instance;

	return instance;
}

HAL_StatusTypeDef Logger::Init() {
	if (__GPIOC_IS_CLK_DISABLED())
		__GPIOC_CLK_ENABLE();

	if (__GPIOD_IS_CLK_DISABLED())
		__GPIOD_CLK_ENABLE();

	if (__UART5_IS_CLK_DISABLED())
		__UART5_CLK_ENABLE();

	if (__DMA1_IS_CLK_DISABLED())
		__DMA1_CLK_ENABLE();

	// PC12 TX
	// PD2 RX
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	hdma_uart5_tx.Instance = DMA1_Stream7;
	hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart5_tx.Init.Mode = DMA_NORMAL;
	hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_uart5_tx))
		return HAL_ERROR;

	__HAL_LINKDMA(&huart, hdmatx, hdma_uart5_tx);


	this->huart.Instance = UART5;
	this->huart.Init.BaudRate = 1500000;
	this->huart.Init.WordLength = UART_WORDLENGTH_8B;
	this->huart.Init.StopBits = UART_STOPBITS_1;
	this->huart.Init.Parity = UART_PARITY_NONE;
	this->huart.Init.Mode = UART_MODE_TX_RX;
	this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	this->huart.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&this->huart))
		return HAL_ERROR;

	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART5_IRQn);

	return HAL_OK;
}

HAL_StatusTypeDef Logger::Print(uint8_t *data, uint16_t len) {
	return HAL_UART_Transmit_DMA(&this->huart, data, len);
}

}
