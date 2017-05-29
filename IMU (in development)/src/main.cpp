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

using namespace flyhero;

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

	void TIM5_IRQHandler(void)
		{
			TIM_HandleTypeDef *htim5 = Timer::Get_Handle();

			// Channel 2 for HAL 1 ms tick
			if (__HAL_TIM_GET_ITSTATUS(htim5, TIM_IT_CC2) == SET) {
				__HAL_TIM_CLEAR_IT(htim5, TIM_IT_CC2);
				uint32_t val = __HAL_TIM_GetCounter(htim5);
				//if ((val - prev) >= 1000) {
					HAL_IncTick();
					// Prepare next interrupt
					__HAL_TIM_SetCompare(htim5, TIM_CHANNEL_2, val + 1000);
					//prev = val;
				//}
				//else {
				//	printf("should not happen\n");
				//}
			}
			//HAL_TIM_IRQHandler(Timer::Get_Handle());
		}

}

int main(void)
{
	HAL_Init();

	initialise_monitor_handles();

	LEDs::Init();

	HAL_Delay(1000);

	if (mpu->Init(false)) {
		LEDs::TurnOn(LEDs::Orange);

		while (true) {
			LEDs::Toggle(LEDs::Orange | LEDs::Yellow);
			HAL_Delay(500);
		}
	}


	printf("init complete\n");

	MPU6050::Sensor_Data gyro, accel;

	uint32_t ppos = 0;
	double p[100][6];

	while (true) {
		if (data_ready) {
			data_ready = false;
			mpu->Read_Raw(&gyro, &accel);
			p[ppos][0] = gyro.x;
			p[ppos][1] = gyro.y;
			p[ppos][2] = gyro.z;
			p[ppos][3] = accel.x;
			p[ppos][4] = accel.y;
			p[ppos][5] = accel.z;

			ppos++;

			if (ppos == 100) {
				printf("a");
			}
		}
	}
}
