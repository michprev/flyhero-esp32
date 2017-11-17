#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"


using namespace flyhero;


void imu_task(void *args);


extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 0);

    while (true);
}

void imu_task(void *args)
{
    IMU *imu = NULL;
    ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

    imu->Init();

    IMU::Sensor_Data accel, gyro;

    while (true)
    {
        if (imu->Data_Ready())
        {
            imu->Read_Data(accel, gyro);
        }
    }
}