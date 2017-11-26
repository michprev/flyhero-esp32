#include <iostream>
#include <nvs_flash.h>

#include "WiFi_Controller.h"


using namespace flyhero;

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_status);

    WiFi_Controller &wifi = WiFi_Controller::Instance();
    LEDs::Init();

    const uint8_t TCP_BUFFER_LENGTH = 50;
    uint8_t TCP_buffer[TCP_BUFFER_LENGTH];
    uint8_t received_length = 0;
    bool process_tcp = true;

    wifi.Init();

    ESP_ERROR_CHECK(wifi.TCP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Wait_For_Client());

    while (process_tcp)
    {
        if (wifi.TCP_Receive(TCP_buffer, TCP_BUFFER_LENGTH, &received_length))
        {
            uint8_t data[] = { 0x01, 0x03, 0x08 };

            wifi.TCP_Send(data, 3);

            printf("Command: ");

            for (uint8_t i = 0; i < received_length; i++)
                printf("0x%x ", TCP_buffer[i]);
            printf("\n");


            if (TCP_buffer[0] == 0x01)
                process_tcp = false;
        }
    }

    ESP_ERROR_CHECK(wifi.UDP_Server_Start());

    WiFi_Controller::In_Datagram_Data data;

    while (true)
    {
        if (wifi.UDP_Receive(data))
        {
            std::cout << data.throttle << " " << data.roll_kp << " " << data.pitch_kp << " " << data.yaw_kp << std::endl;
        }
    }
}
