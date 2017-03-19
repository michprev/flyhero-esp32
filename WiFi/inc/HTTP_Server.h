/*
 * HTTPServer.h
 *
 *  Created on: 16. 3. 2017
 *      Author: michp
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <stm32f4xx_hal.h>

class TCP_Connection;

enum HTTP_State { HTTP_READY, HTTP_HEADER_SENDING, HTTP_HEADER_SENT, HTTP_BODY_SENDING };

class HTTP_Server {
private:
	char send_buffer[1024];
	TCP_Connection *active_connection;
	uint8_t link_ID;
	const char *header;
	const char *body;
	uint16_t body_size;
	HTTP_State state;

public:
	HTTP_Server();

	HTTP_State Get_State();
	HAL_StatusTypeDef HTTP_Send_Begin(uint8_t link_ID, const char *header, const char *body, uint16_t body_size);
	HAL_StatusTypeDef HTTP_Send_Continue();
};

#endif /* HTTP_SERVER_H_ */
