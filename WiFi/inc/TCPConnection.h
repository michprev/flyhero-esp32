/*
 * TCPConnection.h
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#ifndef TCPCONNECTION_H_
#define TCPCONNECTION_H_

#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

class ESP32;

class TCP_Connection {
private:
	enum State { Command, Data, Nothing };

	bool connected;
	bool closed;
	bool reset;
	State state;

	HAL_StatusTypeDef send_packet(char *command, char *data, uint16_t data_size);


public:
	TCP_Connection();
	void Connected();
	void Closed();
	HAL_StatusTypeDef Send_File(char *send_buffer, uint8_t link_ID, const char *header, const char *body, uint16_t body_size);

};

#endif /* TCPCONNECTION_H_ */
