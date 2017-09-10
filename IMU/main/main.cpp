#include "MPU9250.h"

// debug
#include <iostream>
#include <sys/time.h>

using namespace flyhero;

void imu_task(void *args) {
	MPU9250& mpu = MPU9250::Instance();
	mpu.Init();

	timeval time;
	uint32_t tmp[10];
	uint16_t pos = 0;

	bool run = true;

	while (run) {
		if (mpu.Data_Ready()) {
			gettimeofday(&time, NULL);

			tmp[pos] = time.tv_usec - tmp[pos];

			pos++;

			if (pos == 10)
				break;

			tmp[pos] = time.tv_usec;
		}
	}

	for (int i = 0; i < 10; i++)
		std::cout << tmp[i] << std::endl;

	vTaskDelete(NULL);
}

extern "C" void app_main(void) {

	xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 1, NULL, 1);

}
