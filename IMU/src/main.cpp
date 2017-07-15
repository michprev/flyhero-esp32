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

void IMU_Data_Ready_Callback();
void IMU_Data_Read_Callback();

MPU6050& mpu = MPU6050::Instance();
Logger& logger = Logger::Instance();

volatile bool log_flag = false;

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

	logger.Set_Data_Type(Logger::UART, Logger::Accel_All | Logger::Gyro_All | Logger::Euler_All);

	mpu.Data_Ready_Callback = &IMU_Data_Ready_Callback;
	mpu.Data_Read_Callback = &IMU_Data_Read_Callback;

	while (true) {
		if (log_flag) {
			log_flag = false;
			logger.Send_Data();
		}
	}
}

void IMU_Data_Ready_Callback() {
	// 160 us
	if (mpu.Start_Read() != HAL_OK)
		LEDs::TurnOn(LEDs::Orange);
}

// IMU_Data_Ready_Callback() -> 340 us -> IMU_Data_Read_Callback()

void IMU_Data_Read_Callback() {
	mpu.Complete_Read();
	// 200 us
	mpu.Compute_Euler();

	log_flag = true;
}
