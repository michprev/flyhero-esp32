/*
 * Logger.cpp
 *
 *  Created on: 17. 9. 2017
 *      Author: michp
 */

#include "Logger.h"

namespace flyhero {

Logger& Logger::Instance() {
	static Logger instance;

	return instance;
}

Logger::Logger() {
	this->log_file = NULL;
}

esp_err_t Logger::print_and_remove() {
	struct stat s;

	// file exists
	if (stat(this->LOG_FILENAME, &s) == 0) {
		this->log_file = fopen(this->LOG_FILENAME, "r");

		if (this->log_file == NULL)
			return ESP_FAIL;

		int8_t buf;


		while ( (buf = getc(this->log_file)) != EOF) {
			std::cout << buf;
		}

		if (fclose(this->log_file) != 0)
			return ESP_FAIL;

		if (remove(this->LOG_FILENAME) != 0)
			return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t Logger::append_message(uint32_t time, const char *tag, const char *message) {
	this->log_file = fopen(this->LOG_FILENAME, "a");

	std::cout << "file is NULL: " << (this->log_file == NULL ? "true" : "false") << std::endl;

	if (this->log_file != NULL)
		fclose(this->log_file);

	/*if (this->log_file == NULL)
		return ESP_FAIL;

	fprintf(this->log_file, "(%d) %s: %s\n", time, tag, message);

	if (fclose(this->log_file) != 0)
		return ESP_FAIL;*/

	return ESP_OK;
}

void Logger::Init() {
	esp_vfs_spiffs_conf_t conf;
	conf.base_path = "/spiffs";
	conf.format_if_mount_failed = true;
	conf.max_files = 1;
	conf.partition_label = NULL;

	ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

	ESP_ERROR_CHECK(this->print_and_remove());
}

void Logger::Log(const char *tag, const char *message) {
	timeval now;
	gettimeofday(&now, NULL);

	ESP_ERROR_CHECK(this->append_message(now.tv_sec * 1000000 + now.tv_usec, tag, message));
}

} /* namespace flyhero */
