#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "MPU9250.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"

// debug
#include <iostream>
#include <sys/time.h>

using namespace flyhero;

QueueHandle_t data_queue;

void imu_task(void *args) {
	MPU9250& mpu = MPU9250::Instance();
	mpu.Init();

	//Mahony_Filter mahony(2, 0.1f, 1000);
	Complementary_Filter complementary(0.995f, 1000);

	IMU::Sensor_Data accel, gyro;
	IMU::Euler_Angles euler;

	uint8_t i = 0;

	while (true) {
		if (mpu.Data_Ready()) {
			mpu.Read_Data(accel, gyro);

			//mahony.Compute(accel, gyro, euler);
			complementary.Compute(accel, gyro, euler);

			i++;

			if (i % 100 == 0) {
				i = 0;

				if (xQueueSendToBack(data_queue, &euler, 0) != pdTRUE)
					break;
			}
		}
	}

	std::cout << "exited" << std::endl;

	vTaskDelete(NULL);
}

extern "C" void app_main(void) {

	data_queue = xQueueCreate(10, sizeof(IMU::Euler_Angles));

	xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 1, NULL, 1);

	IMU::Euler_Angles euler;

	while (true) {
		if (xQueueReceive(data_queue, &euler, 0) == pdTRUE)
			std::cout << euler.roll << " " << euler.pitch << " " << euler.yaw << std::endl;
	}

}
