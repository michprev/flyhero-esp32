/*
 * WiFi_Controller.h
 *
 *  Created on: 13. 9. 2017
 *      Author: michp
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <lwip/sockets.h>
#include <cstring>

#include "LEDs.h"
#include "CRC.h"
#include "IMU.h"


#define FUSION_ALGORITHMS_USED 2

namespace flyhero
{

class WiFi_Controller
{
public:
    struct __attribute__((__packed__)) In_Datagram_Data
    {
        uint16_t throttle;
        uint16_t roll_kp;
        uint16_t pitch_kp;
        uint16_t yaw_kp;
    };

    struct __attribute__((__packed__)) Out_Datagram_Data
    {
        uint16_t free_time;

        IMU::Euler_Angles euler[FUSION_ALGORITHMS_USED];
    };

private:
    WiFi_Controller();
    WiFi_Controller(WiFi_Controller const &);
    WiFi_Controller &operator=(WiFi_Controller const &);

    static const uint8_t IN_DATAGRAM_LENGTH = 11;
    static const uint8_t OUT_DATAGRAM_LENGTH = 1 + 2 + 12 * FUSION_ALGORITHMS_USED + 2;

    union in_datagram
    {
        uint8_t raw_data[IN_DATAGRAM_LENGTH];

        struct __attribute__((__packed__))
        {
            uint8_t datagram_length;

            In_Datagram_Data datagram_contents;

            uint16_t crc;

        } data;
    };

    union out_datagram
    {
        uint8_t raw_data[OUT_DATAGRAM_LENGTH];

        struct __attribute__((__packed__))
        {
            uint8_t datagram_length;

            Out_Datagram_Data datagram_contents;

            uint16_t crc;

        } data;
    };

    static const uint8_t TCP_BUFFER_LENGTH = 50;
    uint8_t tcp_buffer[TCP_BUFFER_LENGTH];

    const uint16_t UDP_PORT = 4789;
    const uint16_t TCP_PORT = 4821;

    int udp_server_fd;
    int tcp_server_fd;
    int tcp_client_fd;

    sockaddr_in udp_client_socket;
    sockaddr_in tcp_client_address;

    uint32_t udp_client_socket_length;
    uint32_t tcp_client_address_length;

    bool client_connected;

    void ap_init();

public:
    static WiFi_Controller &Instance();

    void Init();
    esp_err_t UDP_Server_Start();
    esp_err_t UDP_Server_Stop();
    esp_err_t TCP_Server_Start();
    esp_err_t TCP_Server_Stop();
    esp_err_t TCP_Wait_For_Client();

    bool UDP_Receive(In_Datagram_Data &datagram_data);
    bool UDP_Send(Out_Datagram_Data datagram_data);
    bool TCP_Receive(char *buffer, uint8_t buffer_length, uint8_t *received_length);
    bool TCP_Send(const char *data, uint8_t data_length);

    void Client_Connected_Callback();
    void Client_Disconnected_Callback();

};

} /* namespace flyhero */
