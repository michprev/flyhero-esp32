/*
 * ESP32.h
 *
 *  Created on: 10. 3. 2017
 *      Author: michp
 */

#ifndef ESP32_H_
#define ESP32_H_

#include <stm32f4xx_hal.h>
#include "TCPConnection.h"

//class TCP_Connection;

class ESP32 {
private:
	/* Singleton */
	ESP32();
	ESP32(ESP32 const&){};
	ESP32& operator=(ESP32 const&){};
	static ESP32* pInstance;

	static const uint32_t BUFFER_SIZE = 8192;
	static const uint32_t IPD_BUFFER_SIZE = 1024;
	static const uint16_t MAX_PARSE_SIZE = 500;
	static const uint16_t UART_TIMEOUT = 1000;
	static const uint8_t MAX_NULL_BYTES = 5;

	struct ReadPos {
		uint32_t pos;

		uint32_t add(uint16_t c) {
			return ((this->pos + c) % BUFFER_SIZE);
		}

		uint32_t previous() {
			return (this->pos == 0 ? BUFFER_SIZE - 1 : this->pos - 1);
		}
	};

	enum ESP_State { ESP_SENDING, ESP_READY, ESP_AWAITING_BODY, ESP_ERROR };


	TCP_Connection TCP_connections[5];
	char send_buffer[2048];
	int8_t link_ID;

	uint32_t timestamp;

	ESP_State state;
	bool wait_for_wrap;
	bool ready;
	bool inIPD;
	uint32_t IPD_received;
	uint32_t IPD_size;
	ReadPos readPos;
	uint16_t processedLength;
	DMA_HandleTypeDef hdma_usart3_rx;
	DMA_HandleTypeDef hdma_usart3_tx;
	UART_HandleTypeDef huart;
	uint8_t IPD_buffer[IPD_BUFFER_SIZE];
	uint8_t buffer[BUFFER_SIZE];
	uint8_t processing_buffer[MAX_PARSE_SIZE];

	HAL_StatusTypeDef UART_Init();
	HAL_StatusTypeDef UART_DMA_send(uint8_t *data, uint16_t size);
	void parse(char *str, uint16_t length);
	bool next_bytes_null();

public:
	static ESP32* Instance();

	void(*IPD_Callback)(uint8_t linkID, uint8_t *data, uint16_t length);

	DMA_HandleTypeDef* Get_DMA_Tx_Handle();
	DMA_HandleTypeDef* Get_DMA_Rx_Handle();
	UART_HandleTypeDef* Get_UART_Handle();
	HAL_StatusTypeDef Init();
	void Process_Data();
	void Set_Wait_For_Wrap(bool value);
	bool Get_Wait_For_Wrap();
	HAL_StatusTypeDef Send(const char *command);
	HAL_StatusTypeDef Send(uint8_t *data, uint16_t count);
	HAL_StatusTypeDef HTTP_Send_File(uint8_t link_ID, const char *header, const char *body, uint16_t body_size);
	HAL_StatusTypeDef TCP_Send(uint8_t link_ID, uint8_t *data, uint16_t data_size);
};

#endif /* ESP32_H_ */
