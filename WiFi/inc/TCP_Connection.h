/*
 * TCPConnection.h
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#ifndef TCP_CONNECTION_H_
#define TCP_CONNECTION_H_

#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

enum TCP_State { TCP_CLOSED, TCP_READY, TCP_COMMAND_SENDING, TCP_COMMAND_SENT, TCP_WAITING_FOR_WRAP, TCP_WRAP_RECEIVED, TCP_DATA_SENDING, TCP_DATA_SENT };

class TCP_Connection {
private:
	bool connected;
	bool closed;
	bool reset;
	TCP_State state;
	uint16_t copied;
	uint16_t data_size;
	char command[30];
	uint8_t *data;
	uint8_t link_ID;
	uint16_t packet_size;

public:
	TCP_Connection();
	void Connected();
	void Closed();
	void Reset();
	TCP_State Get_State();
	HAL_StatusTypeDef TCP_Send_Begin(uint8_t link_ID, uint8_t *data, uint16_t data_size);
	HAL_StatusTypeDef TCP_Send_Continue();
};

#endif /* TCP_CONNECTION_H_ */
