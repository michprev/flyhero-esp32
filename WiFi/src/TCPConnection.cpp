/*
 * TCPConnection.cpp
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#include <TCPConnection.h>
#include "ESP32.h"

TCP_Connection::TCP_Connection() {
	this->closed = false;
	this->connected = false;
	this->reset = false;
	this->state = Nothing;
}

void TCP_Connection::Connected() {
	this->closed = false;
	this->connected = true;
}

void TCP_Connection::Closed() {
	this->connected = false;
	this->closed = true;
	this->reset = true;
}

HAL_StatusTypeDef TCP_Connection::Send_File(char *send_buffer, uint8_t link_ID, const char *header, const char *body, uint16_t body_size) {
	this->reset = false;

	//TODO assuming that header has not more than 2048 characters
	uint16_t copied = 0;
	char command[30];
	sprintf(send_buffer, "%s%d\r\n\r\n", header, body_size);

	uint16_t headerSize = strlen(send_buffer);

	if (body_size != 0) {
		copied += (2048 - headerSize < body_size ? 2048 - headerSize : body_size);
		memcpy(send_buffer + headerSize, body, copied);
	}

	sprintf(command, "AT+CIPSEND=%c,%d\r\n", link_ID, headerSize + copied);
	if (send_packet(command, send_buffer, headerSize + copied) == HAL_ERROR)
		return HAL_ERROR;

	while (body_size - copied >= 2048) {
		sprintf(command, "AT+CIPSEND=%c,2048\r\n", link_ID);
		memcpy(send_buffer, body + copied, 2048);

		if (send_packet(command, send_buffer, 2048) == HAL_ERROR)
			return HAL_ERROR;
		copied += 2048;
	}

	if (body_size - copied != 0) {
		sprintf(command, "AT+CIPSEND=%c,%d\r\n", link_ID, body_size - copied);
		memcpy(send_buffer, body + copied, body_size - copied);

		if (this->send_packet(command, send_buffer, body_size - copied) == HAL_ERROR)
			return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef TCP_Connection::send_packet(char * command, char *data, uint16_t dataSize)
{
	if (this->reset)
		return HAL_ERROR;

	ESP32 *esp = ESP32::Instance();

	esp->Set_Wait_For_Wrap(true);

	this->state = Command;

	esp->Send(command);

	if (this->reset)
		return HAL_ERROR;

	while (esp->Get_Wait_For_Wrap())
		esp->Process_Data();

	if (this->reset)
		return HAL_ERROR;

	this->state = Data;

	esp->Send(data, dataSize);

	if (this->reset)
		return HAL_ERROR;

	return HAL_OK;
}
