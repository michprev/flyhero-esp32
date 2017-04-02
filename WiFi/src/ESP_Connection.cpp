/*
 * ESP_Connection.cpp
 *
 *  Created on: 12. 3. 2017
 *      Author: michp
 */

#include <ESP_Connection.h>
#include "ESP.h"

namespace The_Eye {

ESP_Connection::ESP_Connection(ESP *esp, uint8_t link_ID) : LINK_ID(link_ID) {
	this->esp = esp;
	this->closed = false;
	this->connected = false;
	this->reset = false;
}

void ESP_Connection::Connected() {
	this->closed = false;
	this->connected = true;
}

void ESP_Connection::Closed() {
	this->connected = false;
	this->closed = true;
	this->reset = true;
}

void ESP_Connection::Reset() {
	this->state = CONNECTION_READY;
}

Connection_State ESP_Connection::Get_State() {
	return this->state;
}

HAL_StatusTypeDef ESP_Connection::Connection_Send_Begin(uint8_t *data, uint16_t data_size) {
	if (data_size == 0)
		return HAL_OK;
	this->copied = 0;
	this->reset = false;
	this->data = data;
	this->data_size = data_size;

	if (data_size >= 2048) {
		sprintf(this->command, "AT+CIPSEND=%c,2048\r\n", this->LINK_ID);
		this->packet_size = 2048;
	}
	else {
		sprintf(this->command, "AT+CIPSEND=%c,%d\r\n", this->LINK_ID, data_size);
		this->packet_size = data_size;
	}

	this->state = CONNECTION_COMMAND_SENDING;
	this->esp->Set_Wait_For_Wrap(true);
	return this->esp->Send_Begin(this->command);
}

// TODO handle reset
HAL_StatusTypeDef ESP_Connection::Connection_Send_Continue() {
	switch (this->state) {
	case CONNECTION_READY:
		return HAL_OK;
	case CONNECTION_COMMAND_SENDING:
		this->esp->Process_Data();

		if (this->esp->Get_State() == ESP_READY)
			this->state = CONNECTION_WAITING_FOR_WRAP;

		if (this->esp->Get_Wait_For_Wrap() == false)
			this->state = CONNECTION_WRAP_RECEIVED;

		break;
	case CONNECTION_WAITING_FOR_WRAP:
		this->esp->Process_Data();

		if (this->esp->Get_Wait_For_Wrap() == false)
			this->state = CONNECTION_WRAP_RECEIVED;

		break;
	case CONNECTION_WRAP_RECEIVED:
		this->state = CONNECTION_DATA_SENDING;

		this->esp->Send_Begin(this->data + this->copied, this->packet_size);

		break;
	case CONNECTION_DATA_SENDING:
		this->esp->Process_Data();

		if (this->esp->Get_State() == ESP_READY) {
			this->copied += this->packet_size;

			if (this->copied == this->data_size)
				this->state = CONNECTION_READY;
			else
				this->state = CONNECTION_DATA_SENT;
		}
		else if (this->esp->Get_State() == ESP_ERROR /*&& this->reset*/)
			this->state = CONNECTION_CLOSED;

		break;
	case CONNECTION_DATA_SENT:
		if (this->data_size - this->copied >= 2048) {
			sprintf(this->command, "AT+CIPSEND=%c,2048\r\n", this->LINK_ID);
			this->packet_size = 2048;
		}
		else {
			sprintf(this->command, "AT+CIPSEND=%c,%d\r\n", this->LINK_ID, this->data_size - this->copied);
			this->packet_size = this->data_size - this->copied;
		}

		this->state = CONNECTION_COMMAND_SENDING;
		this->esp->Set_Wait_For_Wrap(true);
		return this->esp->Send_Begin(this->command);
	}
}

}
