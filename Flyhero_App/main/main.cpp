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
QueueHandle_t wifi_log_data_queue;

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
    wifi_log_data_queue = xQueueCreate(2, sizeof(WiFi_Controller::Out_Datagram_Data));

    Logger::Instance().Init();
    IMU &imu = IMU_Detector::Detect_IMU();
    imu.Init();

    // Initialize watchdog with 1 sec timeout
    ESP_ERROR_CHECK(esp_task_wdt_init(1, true));

    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);

    while (true);
}

void imu_task(void *args)
{
    WiFi_Controller::Out_Datagram_Data log_data;
    int64_t start = esp_timer_get_time(), end;
    uint8_t i = 0;
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles complementary_euler;

    // Subscribe IMU task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    IMU &imu = IMU_Detector::Detect_IMU();
    Logger &logger = Logger::Instance();
    Complementary_Filter complementary(0.98);

    while (true)
    {
        if (imu.Data_Ready())
        {
            end = esp_timer_get_time();

            esp_task_wdt_reset();

            imu.Read_Data(accel, gyro);

            logger.Log_Next(&accel, sizeof(accel));
            logger.Log_Next(&gyro, sizeof(gyro));

            complementary.Compute(accel, gyro, complementary_euler);

            i++;

            if (i == imu.Get_Sample_Rate() / 10)
            {
                log_data.euler[0] = complementary_euler;

                log_data.free_time = (end - start) * imu.Get_Sample_Rate() * 0.01f;

                xQueueSend(wifi_log_data_queue, &log_data, 0);
                i = 0;
            }


            motors_controller.Update_Motors(complementary_euler, gyro);

            start = esp_timer_get_time();
        }
    }
}

void wifi_task(void *args)
{
    WiFi_Controller &wifi = WiFi_Controller::Instance();
    WiFi_Controller::In_Datagram_Data in_datagram_data;
    WiFi_Controller::Out_Datagram_Data out_datagram_data;

    const uint8_t TCP_BUFFER_LENGTH = 50;
    char TCP_buffer[TCP_BUFFER_LENGTH];
    uint8_t received_length = 0;
    bool process_tcp = true;

    wifi.Init();

    ESP_ERROR_CHECK(wifi.TCP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Wait_For_Client());

    while (process_tcp)
    {
        if (wifi.TCP_Receive(TCP_buffer, TCP_BUFFER_LENGTH, &received_length))
        {
            if (strncmp((const char *) TCP_buffer, "start", 5) == 0)
            {
                IMU_Detector::Detect_IMU().Gyro_Calibrate();
                if (IMU_Detector::Detect_IMU().Start())
                {
                    wifi.TCP_Send("yup", 3);
                    process_tcp = false;
                } else
                    wifi.TCP_Send("nah", 3);

            } else if (strncmp((const char *) TCP_buffer, "calibrate", 9) == 0)
            {
                IMU_Detector::Detect_IMU().Accel_Calibrate();
                wifi.TCP_Send("yup", 3);
            } else if (strncmp((const char *) TCP_buffer, "log", 3) == 0)
            {
                Logger::Instance().Erase();
                Logger::Instance().Enable_Writes();
                wifi.TCP_Send("yup", 3);
            } else
                wifi.TCP_Send("nah", 3);
        }
    }

    // Subscribe WiFi task to watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);
    ESP_ERROR_CHECK(wifi.TCP_Server_Stop());
    ESP_ERROR_CHECK(wifi.UDP_Server_Start());

    while (true)
    {
        if (wifi.UDP_Receive(in_datagram_data))
        {
            esp_task_wdt_reset();

            double rate_parameters[3][3] = {
                    { in_datagram_data.rate_roll_kp * 0.01,  0, 0 },
                    { in_datagram_data.rate_pitch_kp * 0.01, 0, 0 },
                    { in_datagram_data.rate_yaw_kp * 0.01,   0, 0 }
            };

            double stab_parameters[3][3] = {
                    { in_datagram_data.stab_roll_kp * 0.01,  0, 0 },
                    { in_datagram_data.stab_pitch_kp * 0.01, 0, 0 },
                    { in_datagram_data.stab_yaw_kp * 0.01,   0, 0 }
            };

            motors_controller.Set_Throttle(in_datagram_data.throttle);
            motors_controller.Set_PID_Constants(Motors_Controller::RATE, rate_parameters);
            motors_controller.Set_PID_Constants(Motors_Controller::STABILIZE, stab_parameters);
        }

        if (xQueueReceive(wifi_log_data_queue, &out_datagram_data, 0) == pdTRUE)
        {
            wifi.UDP_Send(out_datagram_data);
        }
    }
}
