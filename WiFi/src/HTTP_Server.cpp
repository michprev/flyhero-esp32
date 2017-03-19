/*
 * HTTPServer.cpp
 *
 *  Created on: 16. 3. 2017
 *      Author: michp
 */

#include <ESP32.h>
#include <HTTP_Server.h>
#include <TCP_Connection.h>

HTTP_Server::HTTP_Server() {
	this->state = HTTP_READY;
}

HTTP_State HTTP_Server::Get_State() {
	return this->state;
}

HAL_StatusTypeDef HTTP_Server::HTTP_Send_Begin(uint8_t link_ID, const char *header, const char *body, uint16_t body_size) {
	sprintf(this->send_buffer, "%s%d\r\n\r\n", header, body_size);

	this->link_ID = link_ID;
	this->header = header;
	this->body = body;
	this->body_size = body_size;
	this->active_connection = ESP32::Instance()->Get_TCP_Connection(link_ID);
	this->state = HTTP_HEADER_SENDING;

	this->active_connection->TCP_Send_Begin(link_ID, (uint8_t*)this->send_buffer, strlen(this->send_buffer));


	// not needed; it wll never take just one step
	/*if (this->active_connection->Get_State() == TCP_Ready)
		this->state = HTTP_Header_Sent;*/

	return HAL_OK;
}

HAL_StatusTypeDef HTTP_Server::HTTP_Send_Continue() {
	if (this->active_connection->Get_State() == TCP_CLOSED) {
		this->state = HTTP_READY;
		this->active_connection->Reset();
	}

	switch (this->state) {
	case HTTP_READY:
		return HAL_OK;
	case HTTP_HEADER_SENDING:
		this->active_connection->TCP_Send_Continue();

		if (this->active_connection->Get_State() == TCP_READY)
			this->state = HTTP_HEADER_SENT;

		break;
	case HTTP_HEADER_SENT:
		if (this->body_size != 0) {
			this->state = HTTP_BODY_SENDING;
			this->active_connection->TCP_Send_Begin(this->link_ID, (uint8_t*)this->body, this->body_size);
		}
		else
			this->state = HTTP_READY;

		break;
	case HTTP_BODY_SENDING:
		this->active_connection->TCP_Send_Continue();

		if (this->active_connection->Get_State() == TCP_READY)
			this->state = HTTP_READY;

		break;
	};
}
