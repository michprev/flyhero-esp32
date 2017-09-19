/*
 * Logger.h
 *
 *  Created on: 17. 9. 2017
 *      Author: michp
 */

#pragma once

#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <iostream>

extern "C" {

#include <esp_spiffs.h>

}

namespace flyhero {

class Logger {
private:
	Logger();
	Logger(Logger const&);
	Logger& operator=(Logger const&);

	const char *LOG_FILENAME = "/spiffs/log.txt";

	FILE *log_file;

	esp_err_t print_and_remove();
	esp_err_t append_message(uint32_t time, const char *tag, const char *message);

public:
	static Logger& Instance();

	void Init();
	void Log(const char *tag, const char *message);
};

} /* namespace flyhero */
