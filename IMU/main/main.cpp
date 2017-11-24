#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_heap_trace.h>

#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"


using namespace flyhero;


void imu_task(void *args);

#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 0);

    while (true);
}

void imu_task(void *args)
{
    ESP_ERROR_CHECK( heap_trace_init_standalone(trace_record, NUM_RECORDS) );

    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_ALL));

    IMU *imu = NULL;
    ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

    ESP_ERROR_CHECK( heap_trace_stop() );
    heap_trace_dump();

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