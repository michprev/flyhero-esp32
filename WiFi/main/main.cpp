#include <iostream>
#include <nvs_flash.h>

#include "WiFi_Controller.h"

using namespace flyhero;

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_status);

	WiFi_Controller& wifi = WiFi_Controller::Instance();
	LEDs::Init();

	wifi.Init();

	uint8_t buffer[200];
	uint8_t read;

	while (true) {
		if (wifi.Receive(buffer, 200, read)) {
			buffer[read] = '\0';

			std::cout << buffer << std::endl;
		}

		vTaskDelay(500 / portTICK_RATE_MS);
	}
}
