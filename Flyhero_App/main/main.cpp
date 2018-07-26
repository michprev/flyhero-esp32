#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>

#include "Motors_Controller.h"
#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"
#include "Logger.h"


using namespace flyhero;

Motors_Controller &motors_controller = Motors_Controller::Instance();

void wifi_task(void *args);

void imu_task(void *args);

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    LEDs::Init();
    motors_controller.Init();

    Logger::Instance().Init();
    IMU &imu = IMU_Detector::Detect_IMU();
    imu.Init();

    // Initialize watchdog with 1 sec timeout
    ESP_ERROR_CHECK(esp_task_wdt_init(1, true));

    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
}

void imu_task(void *args)
{
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles complementary_euler;
    IMU::Read_Data_Type data_type;

    // Subscribe IMU task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    IMU &imu = IMU_Detector::Detect_IMU();
    Logger &logger = Logger::Instance();
    Complementary_Filter complementary(0.97f);

    while (true)
    {
        if (imu.Data_Ready())
        {
            esp_task_wdt_reset();

            data_type = imu.Read_Data(accel, gyro);

            complementary.Compute(accel, gyro, complementary_euler);

            if (data_type == IMU::Read_Data_Type::ACCEL_GYRO)
            {
                logger.Log_Next(&accel, sizeof(accel));
                logger.Log_Next(&gyro, sizeof(gyro));

                motors_controller.Feed_Stab_PIDs(complementary_euler);
                motors_controller.Feed_Rate_PIDs(gyro);
            } else
                motors_controller.Feed_Rate_PIDs(gyro);
        }
    }
}

void wifi_task(void *args)
{
    WiFi_Controller &wifi = WiFi_Controller::Instance();
    WiFi_Controller::In_Datagram_Data in_datagram_data;

    const char * received_data;
    size_t received_length;
    bool process_tcp = true;

    wifi.Init();

    ESP_ERROR_CHECK(wifi.TCP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Wait_For_Client());

    while (process_tcp)
    {
        if (wifi.TCP_Receive(received_data, received_length))
        {
            if (strncmp(received_data, "start", 5) == 0)
            {
                IMU_Detector::Detect_IMU().Gyro_Calibrate();
                if (IMU_Detector::Detect_IMU().Start())
                {
                    ESP_ERROR_CHECK(wifi.UDP_Server_Start());
                    wifi.TCP_Send("yup", 3);
                    process_tcp = false;
                } else
                    wifi.TCP_Send("nah", 3);

            } else if (strncmp(received_data, "calibrate", 9) == 0)
            {
                IMU_Detector::Detect_IMU().Accel_Calibrate();
                wifi.TCP_Send("yup", 3);
            } else if (strncmp(received_data, "log", 3) == 0)
            {
                Logger::Instance().Erase();
                Logger::Instance().Enable_Writes();
                wifi.TCP_Send("yup", 3);
            } else if (strncmp(received_data, "download", strlen("download")) == 0)
            {
                char buffer[1024];
                if (!Logger::Instance().Read_Next(buffer, 1024))
                {
                    wifi.TCP_Send("nah", 3);
                    continue;
                }

                bool empty = true;

                for (uint8_t i = 0; i < 100; i++)
                {
                    if (buffer[i] != 0xFF)
                    {
                        empty = false;
                        break;
                    }
                }

                if (empty)
                {
                    wifi.TCP_Send("nah", 3);
                    continue;
                }

                wifi.TCP_Send("yup", 3);

                do
                {
                    wifi.TCP_Send(buffer, 1024);
                    if (!Logger::Instance().Read_Next(buffer, 1024))
                        break;

                    empty = true;

                    for (uint8_t i = 0; i < 100; i++)
                    {
                        if (buffer[i] != 0xFF)
                        {
                            empty = false;
                            break;
                        }
                    }

                } while (!empty);
            } else
                wifi.TCP_Send("nah", 3);
        }
    }
    ESP_ERROR_CHECK(wifi.TCP_Server_Stop());

    // Subscribe WiFi task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 20, NULL, 1);

    while (true)
    {
        if (wifi.UDP_Receive(in_datagram_data))
        {
            esp_task_wdt_reset();

            float rate_parameters[3][3] = {
                    { in_datagram_data.rate_roll_kp * 0.01f,  0, 0 },
                    { in_datagram_data.rate_pitch_kp * 0.01f, 0, 0 },
                    { in_datagram_data.rate_yaw_kp * 0.01f,   0, 0 }
            };

            float stab_parameters[3][3] = {
                    { in_datagram_data.stab_roll_kp * 0.01f,  in_datagram_data.stab_roll_ki * 0.01f,  0 },
                    { in_datagram_data.stab_pitch_kp * 0.01f, in_datagram_data.stab_pitch_ki * 0.01f, 0 },
                    { 0,                                      0,                                      0 }
            };

            motors_controller.Set_Throttle(in_datagram_data.throttle);
            motors_controller.Set_PID_Constants(Motors_Controller::RATE, rate_parameters);
            motors_controller.Set_PID_Constants(Motors_Controller::STABILIZE, stab_parameters);
        }
    }
}
