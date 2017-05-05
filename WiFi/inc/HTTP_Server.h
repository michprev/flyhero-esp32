/*
 * HTTPServer.h
 *
 *  Created on: 16. 3. 2017
 *      Author: michp
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <ESP_Connection.h>
#include <stm32f4xx_hal.h>
#include "ESP.h"

namespace flyhero {

enum HTTP_State { HTTP_READY, HTTP_HEADER_SENDING, HTTP_HEADER_SENT, HTTP_BODY_SENDING };

class HTTP_Server {
private:
	ESP *esp;
	char send_buffer[1024];
	ESP_Connection *TCP_connection;
	const char *header;
	const char *body;
	uint16_t body_size;
	HTTP_State state;

public:
	HTTP_Server(ESP *esp);

	HTTP_State Get_State();
	HAL_StatusTypeDef HTTP_Send_Begin(uint8_t link_ID, const char *header, const char *body, uint16_t body_size);
	HAL_StatusTypeDef HTTP_Send_Continue();
};

}

#endif /* HTTP_SERVER_H_ */
