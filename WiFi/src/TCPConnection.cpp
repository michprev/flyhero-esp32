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

HAL_StatusTypeDef TCP_Connection::HTTP_Send_File(char *send_buffer, uint8_t link_ID, const char *header, const char *body, uint16_t body_size) {
	sprintf(send_buffer, "%s%d\r\n\r\n", header, body_size);

	if (this->TCP_Send(link_ID, (uint8_t*)send_buffer, strlen(send_buffer)) == HAL_ERROR)
		return HAL_ERROR;

	if (body_size != 0) {
		if (this->TCP_Send(link_ID, (uint8_t*)body, body_size) == HAL_ERROR)
			return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef TCP_Connection::TCP_Send(uint8_t link_ID, uint8_t *data, uint16_t data_size) {
	uint16_t copied = 0;
	this->reset = false;
	char command[30];

	while (data_size - copied >= 2048) {
		sprintf(command, "AT+CIPSEND=%c,2048\r\n", link_ID);

		if (this->send_packet(command, data + copied, 2048) == HAL_ERROR)
			return HAL_ERROR;
		copied += 2048;
	}

	if (data_size - copied != 0) {
		sprintf(command, "AT+CIPSEND=%c,%d\r\n", link_ID, data_size - copied);

		if (this->send_packet(command, data + copied, data_size - copied) == HAL_ERROR)
			return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef TCP_Connection::send_packet(char *command, uint8_t *data, uint16_t dataSize)
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
