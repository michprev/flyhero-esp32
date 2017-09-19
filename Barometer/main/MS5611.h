/*
 * MS5611.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

namespace flyhero {

class MS5611
{
private:
	MS5611();
	MS5611(MS5611 const&);
	MS5611& operator=(MS5611 const&);

	spi_device_handle_t spi;
	uint16_t c[6];
	uint32_t d1;
	uint32_t d2;

	esp_err_t spi_init();
	esp_err_t reset();
	esp_err_t load_prom();
	esp_err_t read_d1();
	esp_err_t read_d2();

public:
	static MS5611& Instance();

	void Init();
	void Get_Data(int32_t& temperature, int32_t& pressure);
};

}
