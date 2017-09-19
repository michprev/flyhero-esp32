/*
 * main.cpp
 *
 *  Created on: 17. 9. 2017
 *      Author: michp
 */

#include "Logger.h"

using namespace flyhero;

extern "C" void app_main(void) {
	Logger& logger = Logger::Instance();

	logger.Init();

	logger.Log("TEST", "started");

	logger.Log("TEST", "Hello world!");

	esp_vfs_spiffs_unregister(NULL);

	while (true) {

	}
}


