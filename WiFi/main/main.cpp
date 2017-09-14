#include <iostream>
#include "WiFi_Controller.h"

using namespace flyhero;

extern "C" void app_main(void)
{
	WiFi_Controller& wifi = WiFi_Controller::Instance();

	wifi.Init();

	uint8_t buffer[200];
	uint8_t read;

	while (true) {
		if (wifi.Receive(buffer, 200, read)) {
			buffer[read] = '\0';

			std::cout << buffer << std::endl;
		}
	}
}
