#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <nvs_flash.h>

#include "Motors_Controller.h"
#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"
#include "CRC.h"


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
        nvs_status = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_status);

    LEDs::Init();

    motors_controller.Init();

    wifi_log_data_queue = xQueueCreate(2, sizeof(WiFi_Controller::Out_Datagram_Data));

    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 0);

    while (true);
}

void imu_task(void *args)
{
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles mahony_euler, complementary_euler;

    IMU *imu;
    ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

    Mahony_Filter mahony(2, 0.1f, 1000);
    Complementary_Filter complementary(0.995f, 1000);

    imu->Init();

    WiFi_Controller::Out_Datagram_Data log_data;

    timeval start, end;
    uint8_t i = 0;

    while (true)
    {
        if (imu->Data_Ready())
        {
            gettimeofday(&end, NULL);

            imu->Read_Data(accel, gyro);

            mahony.Compute(accel, gyro, mahony_euler);
            complementary.Compute(accel, gyro, complementary_euler);

            i++;

            if (i == 100)
            {
                log_data.euler[0] = mahony_euler;
                log_data.euler[1] = complementary_euler;
                log_data.free_time = end.tv_usec - start.tv_usec;

                xQueueSend(wifi_log_data_queue, &log_data, 0);
                i = 0;
            }


            motors_controller.Update_Motors(mahony_euler);

            gettimeofday(&start, NULL);
        }
    }
}

void wifi_task(void *args)
{
    WiFi_Controller &wifi = WiFi_Controller::Instance();

    WiFi_Controller::In_Datagram_Data in_datagram_data;
    WiFi_Controller::Out_Datagram_Data out_datagram_data;

    wifi.Init();

    while (true)
    {
        if (wifi.Receive(in_datagram_data))
        {
            motors_controller.Set_Throttle(in_datagram_data.throttle);
            motors_controller.Set_PID_Constants(Roll, 1, 0, 0);
            motors_controller.Set_PID_Constants(Pitch, 1, 0, 0);
        }

        if (xQueueReceive(wifi_log_data_queue, &out_datagram_data, 0) == pdTRUE)
        {
            wifi.Send(out_datagram_data);
        }
    }
}
