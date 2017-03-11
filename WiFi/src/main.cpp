/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Michal Prevratil
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stm32f4xx.h>
#include "main.h"
#include "image.h"
			
extern "C" void initialise_monitor_handles(void);

ESP32 *esp = ESP32::Instance();

void IPD_Callback(uint8_t *data, uint16_t length) {
	char header[] = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: image/jpeg\r\nContent-Length: ";
	esp->SendFile(header, image, image_size);
}

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	esp->IPD_Callback = &IPD_Callback;
	esp->Init();


	while (true) {
		esp->ProcessData();
	}
}
