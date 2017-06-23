/*
 * ESP.h
 *
 *  Created on: 1. 4. 2017
 *      Author: michp
 */

#ifndef ESP_H_
#define ESP_H_

#include <ESP_Connection.h>
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <string.h>

namespace flyhero {

enum ESP_State { ESP_SENDING, ESP_READY, ESP_ERROR };
enum ESP_Device { ESP8266, ESP32, NONE };

class ESP {
private:
	static ESP_Device device;
protected:
	static const uint32_t BUFFER_SIZE = 8192;
	static const uint32_t IPD_BUFFER_SIZE = 1024;
	static const uint16_t MAX_PARSE_SIZE = 500;
	static const uint16_t UART_TIMEOUT = 1000;
	static const uint8_t MAX_NULL_BYTES = 5;

	struct ReadPos {
		uint32_t pos = 0;

		uint32_t add(uint16_t c) {
			return ((this->pos + c) % BUFFER_SIZE);
		}

		uint32_t previous() {
			return (this->pos == 0 ? BUFFER_SIZE - 1 : this->pos - 1);
		}
	};

	ESP_Connection connections[5];
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

	void (*IPD_callback)(uint8_t linkID, uint8_t *data, uint16_t length);

	ESP();
	HAL_StatusTypeDef UART_Init(uint32_t baudrate);
	void reset();
	void parse(char *str, uint16_t length);
	uint32_t bytes_available();

public:
	virtual HAL_StatusTypeDef Init(void (*IPD_callback)(uint8_t linkID, uint8_t *data, uint16_t length)) = 0;

	static ESP* Create_Instance(ESP_Device dev);
	static ESP* Instance();
	ESP_State Get_State();
	ESP_Connection* Get_Connection(uint8_t link_ID);
	DMA_HandleTypeDef* Get_DMA_Tx_Handle();
	DMA_HandleTypeDef* Get_DMA_Rx_Handle();
	UART_HandleTypeDef* Get_UART_Handle();
	void Process_Data();
	void Set_Wait_For_Wrap(bool value);
	bool Get_Wait_For_Wrap();
	HAL_StatusTypeDef Send_Begin(const char *command);
	HAL_StatusTypeDef Send_Begin(uint8_t *data, uint16_t count);
	HAL_StatusTypeDef Send(const char *command);
	HAL_StatusTypeDef Send(uint8_t *data, uint16_t count);
};

} /* namespace flyhero */

#endif /* ESP_H_ */
