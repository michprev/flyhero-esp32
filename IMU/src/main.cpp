/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <MPU6050.h>
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"

using namespace flyhero;

extern "C" void initialise_monitor_handles(void);
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

MPU6050 *mpu = MPU6050::Instance();

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	mpu->dataReady = true;
}

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	printf("init\n");

	IWDG_HandleTypeDef hiwdg;
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 2047;

	/*if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		printf("Could not start watchdog\n");
	}*/

	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	if (__I2C1_IS_CLK_DISABLED())
		__I2C1_CLK_ENABLE();

	I2C_HandleTypeDef I2C_Handle;

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_Handle.Instance = I2C1;
	I2C_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C_Handle.Init.ClockSpeed = 400000;
	I2C_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	I2C_Handle.Init.OwnAddress1 = 0;
	I2C_Handle.Init.OwnAddress2 = 0;

	HAL_I2C_Init(&I2C_Handle);

	uint8_t result;
	if (result = mpu->Init(&I2C_Handle)) {
		printf("Error %d\n", result);
	}

	MPU6050::Sensor_Data data;

	uint32_t t = HAL_GetTick();
	bool run = true;

	while (true)
	{
		//HAL_IWDG_Refresh(&hiwdg);


		if (HAL_GetTick() - t >= 10000 && run) {
			//printf("start\n");
			mpu->SelfTest();
			run = false;
		}

		if (mpu->CheckNewData() && HAL_GetTick() - t >= 20000) {
			mpu->ReadEuler(&data);
			printf("data: %f %f %f\n", data.x, data.y, data.z);
		}
	}
}
