/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Michal Prevratil
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "main.h"
#include <stm32f4xx.h>
#include <string.h>
#include "smoothie.h"
#include "http_parser.h"
			
extern "C" void initialise_monitor_handles(void);

ESP32 *esp = ESP32::Instance();
uint8_t *IPD_data;
uint8_t IPD_link_ID;
uint16_t IPD_length;

http_parser parser;
http_parser_settings settings;

struct Url {
	char address[20];
	uint8_t link_ID;
};

Url url[10];
uint8_t urlRead = 0;
uint8_t urlWrite = 0;
uint8_t link;

int on_url_callback(http_parser *parser, const char *at, size_t length) {
	strncpy((url[urlWrite].address), at, length);
	url[urlWrite].address[length] = '\0';
	url[urlWrite].link_ID = link;

	urlWrite++;
	if (urlWrite == 10)
		urlWrite = 0;

	return 0;
}

void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length) {
	link = link_ID;
	uint16_t parsed = http_parser_execute(&parser, &settings, (char*)data, length);

	if (parsed != length) {
		printf("IPD not parsed\n");
		printf("Parsed %d of %d\n", parsed, length);
	}
}

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	http_parser_init(&parser, HTTP_BOTH);
	settings.on_url = on_url_callback;

	esp->IPD_Callback = &IPD_Callback;
	esp->Init();


	while (true) {
		if (esp->Get_HTTP_Server()->Get_State() == HTTP_READY)
			esp->Process_Data();
		else
			esp->Get_HTTP_Server()->HTTP_Send_Continue();

		while (urlRead != urlWrite && esp->Get_HTTP_Server()->Get_State() == HTTP_READY) {

			if (strcmp(url[urlRead].address, "/") == 0) {
				char body[] = "<!DOCTYPE html> <html> <head> <title>DronUI</title> <meta charset=\"utf-8\" /> <script type=\"text/javascript\" src=\"smoothie.js\"></script> <script> function init() { var tempChart = new SmoothieChart({ interpolation: 'linear' }); var tempLine = new TimeSeries(); tempChart.addTimeSeries(tempLine, { lineWidth: 2, strokeStyle: '#00ff00' }); tempChart.streamTo(document.getElementById(\"tempCanvas\"), 1000); var pressChart = new SmoothieChart({ interpolation: 'linear' }); var pressLine = new TimeSeries(); pressChart.addTimeSeries(pressLine, { lineWidth: 2, strokeStyle: '#00ff00' }); pressChart.streamTo(document.getElementById(\"pressCanvas\"), 1000); setInterval(function () { var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange = function () { if (this.readyState == 4 && this.status == 200) { var data = JSON.parse(this.responseText); tempLine.append(new Date().getTime(), data.temp); pressLine.append(new Date().getTime(), data.press); } }; xhttp.open(\"GET\", \"getData\", true); xhttp.setRequestHeader(\"Connection\", \"Keep-Alive\"); xhttp.send(); }, 1000); } </script> </head> <body onload=\"init()\"> <h2>Ultrasonic sensor</h2> <canvas id=\"tempCanvas\" width=\"900\" height=\"100\"></canvas> <canvas id=\"pressCanvas\" width=\"900\" height=\"100\"></canvas> </body> </html>";
				char header[] = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: text/html\r\nContent-Length: ";

				esp->Get_HTTP_Server()->HTTP_Send_Begin(url[urlRead].link_ID, header, body, strlen(body));
			}
			else if (strcmp(url[urlRead].address, "/smoothie.js") == 0) {
				char header[] = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: application/javascript\r\nContent-Encoding: gzip\r\nContent-Length: ";

				esp->Get_HTTP_Server()->HTTP_Send_Begin(url[urlRead].link_ID, header, smoothie, smoothie_size);
			}
			else if (strcmp(url[urlRead].address, "/favicon.ico") == 0) {
				char header[] = "HTTP/1.1 404 Not Found\r\nConnection: keep-alive\r\nContent-Type: text/html\r\nContent-Length: ";

				esp->Get_HTTP_Server()->HTTP_Send_Begin(url[urlRead].link_ID, header, NULL, 0);
			}
			else if (strcmp(url[urlRead].address, "/getData") == 0) {
				char header[] = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: application/json\r\nContent-Length: ";
				char body[30];

				sprintf(body, "{\r\n\"temp\": %d,\r\n\"press\": %d\r\n}", rand() % 100, rand() % 100);
				esp->Get_HTTP_Server()->HTTP_Send_Begin(url[urlRead].link_ID, header, body, strlen(body));
			}
			else {
				printf("Unhandled URL: %s\n", url[urlRead]);
			}

			urlRead++;
			if (urlRead == 10)
				urlRead = 0;
		}
	}
}
