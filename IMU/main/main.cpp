#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <iostream>

#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"


using namespace flyhero;

QueueHandle_t data_queue;

void imu_task(void *args);
void log_task(void *args);


extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    data_queue = xQueueCreate(10, sizeof(IMU::Euler_Angles));

    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(log_task, "Log task", 4096, NULL, 2, NULL, 0);
}

void imu_task(void *args)
{
    IMU& imu = IMU_Detector::Detect_IMU();

    imu.Init();

    while (!imu.Start())
    {
        std::cout << "Calibrating..." << std::endl;
        imu.Calibrate();
    }

    //Mahony_Filter mahony(100, 0);
    Complementary_Filter complementary(0.98f);

    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles euler;

    uint8_t i = 0;

    while (true)
    {
        if (imu.Data_Ready())
        {
            imu.Read_Data(accel, gyro);

            //mahony.Compute(accel, gyro, euler);
            complementary.Compute(accel, gyro, euler);

            i++;

            if (i == imu.Get_Sample_Rate() / 10)
            {
                i = 0;

                if (xQueueSendToBack(data_queue, &euler, 0) != pdTRUE)
                    break;
            }
        }
    }
}

void log_task(void *args)
{
    IMU::Euler_Angles euler;

    while (true)
    {
        if (xQueueReceive(data_queue, &euler, 0) == pdTRUE)
            std::cout << euler.roll << " " << euler.pitch << " " << euler.yaw << std::endl;
    }
}