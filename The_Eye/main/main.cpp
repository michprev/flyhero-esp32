#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>

#include "Motors_Controller.h"
#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"


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

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    LEDs::Init();
    motors_controller.Init();
    wifi_log_data_queue = xQueueCreate(2, sizeof(WiFi_Controller::Out_Datagram_Data));

    // Initialize watchdog with 5 sec timeout
    if (esp_task_wdt_init(5, true) != ESP_OK)
    {
        while (true)
        {
            LEDs::Turn_On(LEDs::WARNING);

            vTaskDelay(250 / portTICK_RATE_MS);

            LEDs::Turn_Off(LEDs::WARNING);

            vTaskDelay(250 / portTICK_RATE_MS);
        }
    }


    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 0);

    while (true);
}

void imu_task(void *args)
{
    WiFi_Controller::Out_Datagram_Data log_data;
    timeval start, end;
    uint8_t i = 0;
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles mahony_euler, complementary_euler;


    // Subscribe IMU task to watchdog
    if (esp_task_wdt_add(NULL) != ESP_OK)
        vTaskDelete(NULL);

    IMU& imu = IMU_Detector::Detect_IMU();

    Mahony_Filter mahony(100, 0);
    Complementary_Filter complementary(0.98f);

    imu.Init();

    while (true)
    {
        if (imu.Data_Ready())
        {
            esp_task_wdt_reset();

            gettimeofday(&end, NULL);

            imu.Read_Data(accel, gyro);

            mahony.Compute(accel, gyro, mahony_euler);
            complementary.Compute(accel, gyro, complementary_euler);

            i++;

            if (i == 100)
            {
                log_data.euler[0] = mahony_euler;
                log_data.euler[1] = complementary_euler;


                long int delta = (end.tv_usec > start.tv_usec ? end.tv_usec - start.tv_usec
                                                    : 1000000 - end.tv_usec + start.tv_usec);

                log_data.free_time = delta * imu.Get_Sample_Rate() * 0.01f;


                xQueueSend(wifi_log_data_queue, &log_data, 0);
                i = 0;
            }


            motors_controller.Update_Motors(complementary_euler);

            gettimeofday(&start, NULL);
        }
    }
}

void wifi_task(void *args)
{
    WiFi_Controller &wifi = WiFi_Controller::Instance();
    WiFi_Controller::In_Datagram_Data in_datagram_data;
    WiFi_Controller::Out_Datagram_Data out_datagram_data;
    bool connected = false;

    wifi.Init();

    while (true)
    {
        if (wifi.Receive(in_datagram_data))
        {
            if (!connected)
            {
                // Subscribe WiFi task to watchdog
                if (esp_task_wdt_add(NULL) != ESP_OK)
                    vTaskDelete(NULL);

                connected = true;
            }

            esp_task_wdt_reset();

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
