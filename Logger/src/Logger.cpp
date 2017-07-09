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

HAL_StatusTypeDef Logger::Set_Data_Type(Log_Type log_type, Data_Type data_type) {
	this->log_type = log_type;
	this->data_type = data_type;
	this->log = (data_type != 0);

	if (!this->log)
		return HAL_OK;

	if (this->log_type == UART) {
		uint16_t header = 0;

		if (data_type & Accel_X)
			header |= 0x400;
		if (data_type & Accel_Y)
			header |= 0x200;
		if (data_type & Accel_Z)
			header |= 0x100;
		if (data_type & Gyro_X)
			header |= 0x80;
		if (data_type & Gyro_Y)
			header |= 0x40;
		if (data_type & Gyro_Z)
			header |= 0x20;
		if (data_type & Temperature)
			header |= 0x10;
		if (data_type & Roll)
			header |= 0x8;
		if (data_type & Pitch)
			header |= 0x4;
		if (data_type & Yaw)
			header |= 0x2;
		if (data_type & Throttle)
			header |= 0x1;

		uint8_t tmp[3];
		tmp[0] = 0x33;
		tmp[1] = header >> 8;
		tmp[2] = header & 0xFF;

		return this->Print(tmp, 3);
	}
	else if (this->log_type == WiFi)
		return HAL_OK;

	return HAL_ERROR;
}

HAL_StatusTypeDef Logger::Send_Data() {
	if (this->log) {
		uint8_t buffer_pos = 0;
		MPU6050::Raw_Data raw_accel, raw_gyro;
		float roll, pitch, yaw;
		int16_t raw_temp;

		if (this->log_type == WiFi) {
			static uint8_t counter = 0;
			counter++;

			if (counter != 5)
				return HAL_OK;

			counter = 0;
		}

		if ((this->data_type & Accel_All) != 0)
			MPU6050::Instance().Get_Raw_Accel(raw_accel);
		if ((this->data_type & Gyro_All) != 0)
			MPU6050::Instance().Get_Raw_Gyro(raw_gyro);
		if ((this->data_type & Temperature) != 0)
			MPU6050::Instance().Get_Raw_Temp(raw_temp);
		if ((this->data_type & Euler_All) != 0)
			MPU6050::Instance().Get_Euler(roll, pitch, yaw);

		this->data_buffer[0] = 0x33;
		buffer_pos++;

		if (this->data_type & Accel_X) {
			this->data_buffer[buffer_pos] = raw_accel.x >> 8;
			this->data_buffer[buffer_pos + 1] = raw_accel.x & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Accel_Y) {
			this->data_buffer[buffer_pos] = raw_accel.y >> 8;
			this->data_buffer[buffer_pos + 1] = raw_accel.y & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Accel_Z) {
			this->data_buffer[buffer_pos] = raw_accel.z >> 8;
			this->data_buffer[buffer_pos + 1] = raw_accel.z & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Gyro_X) {
			this->data_buffer[buffer_pos] = raw_gyro.x >> 8;
			this->data_buffer[buffer_pos + 1] = raw_gyro.x & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Gyro_Y) {
			this->data_buffer[buffer_pos] = raw_gyro.y >> 8;
			this->data_buffer[buffer_pos + 1] = raw_gyro.y & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Gyro_Z) {
			this->data_buffer[buffer_pos] = raw_gyro.z >> 8;
			this->data_buffer[buffer_pos + 1] = raw_gyro.z & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Temperature) {
			this->data_buffer[buffer_pos] = raw_temp >> 8;
			this->data_buffer[buffer_pos + 1] = raw_temp & 0xFF;

			buffer_pos += 2;
		}
		if (this->data_type & Roll) {
			uint8_t *tmp = (uint8_t*)&roll;
			this->data_buffer[buffer_pos] = tmp[3];
			this->data_buffer[buffer_pos + 1] = tmp[2];
			this->data_buffer[buffer_pos + 2] = tmp[1];
			this->data_buffer[buffer_pos + 3] = tmp[0];

			buffer_pos += 4;
		}
		if (this->data_type & Pitch) {
			uint8_t *tmp = (uint8_t*)&pitch;
			this->data_buffer[buffer_pos] = tmp[3];
			this->data_buffer[buffer_pos + 1] = tmp[2];
			this->data_buffer[buffer_pos + 2] = tmp[1];
			this->data_buffer[buffer_pos + 3] = tmp[0];

			buffer_pos += 4;
		}
		if (this->data_type & Yaw) {
			uint8_t *tmp = (uint8_t*)&yaw;
			this->data_buffer[buffer_pos] = tmp[3];
			this->data_buffer[buffer_pos + 1] = tmp[2];
			this->data_buffer[buffer_pos + 2] = tmp[1];
			this->data_buffer[buffer_pos + 3] = tmp[0];

			buffer_pos += 4;
		}
		if (this->data_type & Throttle) {
			this->data_buffer[buffer_pos] = 0;
			this->data_buffer[buffer_pos + 1] = 0;

			buffer_pos += 2;
			// TODO: implement throttle logging
		}

		this->data_buffer[buffer_pos] = 0;

		for (uint8_t i = 1; i < buffer_pos; i++)
			this->data_buffer[buffer_pos] ^= this->data_buffer[i];

		if (this->log_type == UART)
			return this->Print(this->data_buffer, buffer_pos + 1);
		else if (this->log_type == WiFi)
			return ESP::Instance().Get_Connection('4')->Connection_Send_Begin(this->data_buffer, buffer_pos + 1);

		return HAL_ERROR;
	}
	else
		return HAL_OK;
}

}
