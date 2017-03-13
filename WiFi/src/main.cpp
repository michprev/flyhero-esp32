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
bool IPD_ready = false;
uint8_t *IPD_data;
uint8_t IPD_link_ID;
uint16_t IPD_length;

void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length) {
	IPD_link_ID = link_ID;
	IPD_data = data;
	IPD_length = length;
	IPD_ready = true;
}

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	esp->IPD_Callback = &IPD_Callback;
	esp->Init();


	while (true) {
		esp->Process_Data();

		if (IPD_ready) {
			IPD_ready = false;

			char header[] = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: image/jpeg\r\nContent-Length: ";
			esp->Send_File(IPD_link_ID, header, image, image_size);
		}
	}
}
