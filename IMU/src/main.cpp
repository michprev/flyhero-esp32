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

MPU6050& mpu = MPU6050::Instance();
Logger& logger = Logger::Instance();

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	LEDs::Init();
	logger.Init();

	Timer::Delay_ms(1000);

	if (mpu.Init() || mpu.Calibrate()) {
		while (true) {
			LEDs::Toggle(LEDs::Green);
			Timer::Delay_ms(500);
		}
	}


	printf("init complete\n");

	MPU6050::Raw_Data gyro, accel;
	int16_t temp;

	logger.Set_Data_Type(Logger::UART, Logger::Accel_All | Logger::Gyro_All | Logger::Euler_All);

	mpu.ready = true;

	while (true) {
		if (mpu.Data_Read()) {
			mpu.Complete_Read();
			mpu.Compute_Euler();
			logger.Send_Data();
		}
	}
}
