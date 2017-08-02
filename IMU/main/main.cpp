#include "MPU6050.h"
#include <stdint.h>

using namespace flyhero;

void IMU_Data_Ready_Callback();
void IMU_Data_Read_Callback();

MPU6050& mpu = MPU6050::Instance();

extern "C" void app_main(void)
{
	if (mpu.Init() || mpu.Calibrate()) {
		while (true) {
			/*LEDs::Toggle(LEDs::Green);
			Timer::Delay_ms(500);*/
		}
	}

	printf("init complete\n");

	//mpu.Data_Ready_Callback = &IMU_Data_Ready_Callback;
	//mpu.Data_Read_Callback = &IMU_Data_Read_Callback;
	MPU6050::Raw_Data accel, gyro;

	while (true) {
		mpu.Read_Raw(accel, gyro);

		printf("%d %d %d %d %d %d\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void IMU_Data_Ready_Callback() {
	// 160 us
	/*if (mpu.Start_Read() != HAL_OK)
		LEDs::TurnOn(LEDs::Orange);*/
}

// IMU_Data_Ready_Callback() -> 340 us -> IMU_Data_Read_Callback()

void IMU_Data_Read_Callback() {
	mpu.Complete_Read();

	// 200 us
	mpu.Compute_Mahony();
	//mpu.Compute_Euler();
}
