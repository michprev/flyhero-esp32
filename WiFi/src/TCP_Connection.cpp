/*
 * TCPConnection.cpp
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#include <TCP_Connection.h>
#include "ESP32.h"

TCP_Connection::TCP_Connection() {
	this->closed = false;
	this->connected = false;
	this->reset = false;
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

void TCP_Connection::Reset() {
	this->state = TCP_READY;
}

TCP_State TCP_Connection::Get_State() {
	return this->state;
}

HAL_StatusTypeDef TCP_Connection::TCP_Send_Begin(uint8_t link_ID, uint8_t *data, uint16_t data_size) {
	if (data_size == 0)
		return HAL_OK;
	this->copied = 0;
	this->reset = false;
	this->link_ID = link_ID;
	this->data = data;
	this->data_size = data_size;

	ESP32 *esp = ESP32::Instance();

	if (data_size >= 2048) {
		sprintf(this->command, "AT+CIPSEND=%c,2048\r\n", this->link_ID);
		this->packet_size = 2048;
	}
	else {
		sprintf(this->command, "AT+CIPSEND=%c,%d\r\n", link_ID, data_size);
		this->packet_size = data_size;
	}

	this->state = TCP_COMMAND_SENDING;
	esp->Set_Wait_For_Wrap(true);
	return esp->Send_Begin(this->command);
}

// TODO handle reset
HAL_StatusTypeDef TCP_Connection::TCP_Send_Continue() {
	ESP32 *esp = ESP32::Instance();

	switch (this->state) {
	case TCP_READY:
		return HAL_OK;
	case TCP_COMMAND_SENDING:
		esp->Process_Data();

		if (esp->Get_State() == ESP_READY)
			this->state = TCP_WAITING_FOR_WRAP;

		if (esp->Get_Wait_For_Wrap() == false)
			this->state = TCP_WRAP_RECEIVED;

		break;
	case TCP_WAITING_FOR_WRAP:
		esp->Process_Data();

		if (esp->Get_Wait_For_Wrap() == false)
			this->state = TCP_WRAP_RECEIVED;

		break;
	case TCP_WRAP_RECEIVED:
		this->state = TCP_DATA_SENDING;

		esp->Send_Begin(this->data + this->copied, this->packet_size);

		break;
	case TCP_DATA_SENDING:
		esp->Process_Data();

		if (esp->Get_State() == ESP_READY) {
			this->copied += this->packet_size;

			if (this->copied == this->data_size)
				this->state = TCP_READY;
			else
				this->state = TCP_DATA_SENT;
		}
		else if (esp->Get_State() == ESP_ERROR /*&& this->reset*/)
			this->state = TCP_CLOSED;

		break;
	case TCP_DATA_SENT:
		if (this->data_size - this->copied >= 2048) {
			sprintf(this->command, "AT+CIPSEND=%c,2048\r\n", this->link_ID);
			this->packet_size = 2048;
		}
		else {
			sprintf(this->command, "AT+CIPSEND=%c,%d\r\n", link_ID, this->data_size - this->copied);
			this->packet_size = this->data_size - this->copied;
		}

		this->state = TCP_COMMAND_SENDING;
		esp->Set_Wait_For_Wrap(true);
		return esp->Send_Begin(this->command);
	}
}
