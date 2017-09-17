/*
 * WiFi_Controller.h
 *
 *  Created on: 13. 9. 2017
 *      Author: michp
 */

#pragma once

#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <sys/socket.h>
#include <cstring>
#include "LEDs.h"

namespace flyhero {

class WiFi_Controller {
private:
	WiFi_Controller();
	WiFi_Controller(WiFi_Controller const&);
	WiFi_Controller& operator=(WiFi_Controller const&);

	const uint16_t UDP_PORT = 4789;

	int socket_handle;
	sockaddr_in client;
	uint32_t client_socket_length;
	bool client_connected;

	void ap_init();
	esp_err_t udp_server_start();

public:
	static WiFi_Controller& Instance();

	void Init();
	bool Receive(uint8_t *buffer, uint8_t buffer_length, uint8_t& received_length);
	bool Send(uint8_t *data, uint8_t data_length);
	void Client_Connected_Callback();
	void Client_Disconnected_Callback();

};

} /* namespace flyhero */
