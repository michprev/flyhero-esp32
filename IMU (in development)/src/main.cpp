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

using namespace The_Eye;

extern "C" void initialise_monitor_handles(void);

MPU6050 *mpu = MPU6050::Instance();
volatile bool data_ready = false;

extern "C" {
	void DMA1_Stream5_IRQHandler(void)
	{
		HAL_DMA_IRQHandler(mpu->Get_DMA_Rx_Handle());
	}

	void EXTI1_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	}

	void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		data_ready = true;
	}
}

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();
	if (mpu->Init(false)) {
		LEDs::TurnOn(LEDs::Orange);

		while (true) {
			LEDs::Toggle(LEDs::Orange | LEDs::Yellow);
			HAL_Delay(500);
		}
	}


	printf("init complete\n");

	MPU6050::Sensor_Data accel;

	while (true) {
		if (data_ready) {
			data_ready = false;
			mpu->Read_FIFO();
		}
	}
}
