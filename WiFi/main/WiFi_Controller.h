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

namespace flyhero {

class WiFi_Controller {
private:
	WiFi_Controller();
	WiFi_Controller(WiFi_Controller const&);
	WiFi_Controller& operator=(WiFi_Controller const&);

	const uint16_t UDP_PORT = 4789;

	int socket_handle;

	void ap_init();
	esp_err_t udp_server_start();

public:
	static WiFi_Controller& Instance();

	void Init();
	bool Receive(uint8_t *buffer, uint8_t buffer_length, uint8_t& received_length);

};

} /* namespace flyhero */
