/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "MPU6050.h"
#include "LEDs.h"
#include "Timer.h"
#include "Logger.h"

using namespace flyhero;

extern "C" void initialise_monitor_handles(void);

MPU6050 *mpu = MPU6050::Instance();
Logger *logger = Logger::Instance();

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	LEDs::Init();
	logger->Init();

	HAL_Delay(1000);

	if (mpu->Init() || mpu->Calibrate()) {
		while (true) {
			LEDs::Toggle(LEDs::Green);
			HAL_Delay(500);
		}
	}


	printf("init complete\n");

	MPU6050::Raw_Data gyro, accel;

	uint32_t ppos = 0;
	double p[100][6];

	mpu->ready = true;

	uint32_t ticks = Timer::Get_Tick_Count();
	float roll, pitch, yaw;

	while (true) {
		if (mpu->Data_Ready() && Timer::Get_Tick_Count() - ticks >= 1000000) {
			if (mpu->Start_Read_Raw() != HAL_OK) {
				printf("a");
			}
			ticks = Timer::Get_Tick_Count();
		}
		if (mpu->Data_Read()) {
			mpu->Complete_Read_Raw(&gyro, &accel);

			uint8_t tmp[15];
			tmp[0] = accel.x & 0xFF;
			tmp[1] = accel.x >> 8;
			tmp[2] = accel.y & 0xFF;
			tmp[3] = accel.y >> 8;
			tmp[4] = accel.z & 0xFF;
			tmp[5] = accel.z >> 8;
			tmp[6] = gyro.x & 0xFF;
			tmp[7] = gyro.x >> 8;
			tmp[8] = gyro.y & 0xFF;
			tmp[9] = gyro.y >> 8;
			tmp[10] = gyro.z & 0xFF;
			tmp[11] = gyro.z >> 8;
			tmp[12] = 0;
			tmp[13] = 0;
			tmp[14] = 0;

			for (uint8_t i = 0; i <= 13; i++)
				tmp[14] ^= tmp[i];

			//logger->Print((uint8_t*)"abcd\n", 5);
			//logger->Print(tmp, 15);

			printf("%d %d %d %d %d %d\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
			//mpu->Get_Euler(&roll, &pitch, &yaw);

			//printf("%f %f %f\n", roll, pitch, yaw);
		}
	}
}
