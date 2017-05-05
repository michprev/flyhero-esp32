/*
 * ESP_Connection.h
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#ifndef ESP_CONNECTION_H_
#define ESP_CONNECTION_H_

#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

namespace flyhero {

class ESP;

enum Connection_State { CONNECTION_CLOSED, CONNECTION_READY, CONNECTION_COMMAND_SENDING, CONNECTION_COMMAND_SENT, CONNECTION_WAITING_FOR_WRAP, CONNECTION_WRAP_RECEIVED, CONNECTION_DATA_SENDING, CONNECTION_DATA_SENT };

class ESP_Connection {
private:
	const uint8_t LINK_ID;

	ESP *esp;
	bool connected;
	bool closed;
	bool reset;
	Connection_State state;
	uint16_t copied;
	uint16_t data_size;
	char command[30];
	uint8_t *data;
	uint16_t packet_size;

public:
	ESP_Connection(ESP *esp, uint8_t link_ID);
	void Connected();
	void Closed();
	void Reset();
	Connection_State Get_State();
	HAL_StatusTypeDef Connection_Send_Begin(uint8_t *data, uint16_t data_size);
	HAL_StatusTypeDef Connection_Send_Continue();
};

}

#endif /* ESP_CONNECTION_H_ */
