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
#include "../../Data_Analysis/main/WiFi_Controller.h"


using namespace flyhero;

Motors_Controller &motors_controller = Motors_Controller::Instance();
QueueHandle_t wifi_log_data_queue;
SemaphoreHandle_t imu_task_semaphore;

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
    imu_task_semaphore = xSemaphoreCreateBinary();

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

    vTaskDelay(100 / portTICK_RATE_MS);
    while (xSemaphoreTake(imu_task_semaphore, 0) != pdTRUE);

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
                                                    : 1000000 + end.tv_usec - start.tv_usec);

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
    xSemaphoreTake(imu_task_semaphore, 0);

    WiFi_Controller &wifi = WiFi_Controller::Instance();
    WiFi_Controller::In_Datagram_Data in_datagram_data;
    WiFi_Controller::Out_Datagram_Data out_datagram_data;
    bool connected = false;

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
            if (strncmp((const char*)TCP_buffer, "start", 5) == 0)
            {
                wifi.TCP_Send("yup", 3);
                process_tcp = false;
            } else
                wifi.TCP_Send("nah", 3);
        }
    }

    xSemaphoreGive(imu_task_semaphore);
    ESP_ERROR_CHECK(wifi.TCP_Server_Stop());
    ESP_ERROR_CHECK(wifi.UDP_Server_Start());

    while (true)
    {
        if (wifi.UDP_Receive(in_datagram_data))
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
            motors_controller.Set_PID_Constants(Roll, in_datagram_data.roll_kp * 0.01f, 0, 0);
            motors_controller.Set_PID_Constants(Pitch, in_datagram_data.pitch_kp * 0.01f, 0, 0);
            motors_controller.Set_PID_Constants(Yaw, in_datagram_data.yaw_kp * 0.01f, 0, 0);
        }

        if (xQueueReceive(wifi_log_data_queue, &out_datagram_data, 0) == pdTRUE)
        {
            wifi.UDP_Send(out_datagram_data);
        }
    }
}
