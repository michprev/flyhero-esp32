/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "MPU9250.h"

extern "C" void initialise_monitor_handles(void);
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

MPU9250 *mpu = MPU9250::Instance();

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

	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		printf("Could not start watchdog\n");
	}

	uint8_t result;
	if (result = mpu->Init()) {
		printf("Error %d\n", result);
	}

	float data[3];
	uint8_t accuracy;

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

		if (mpu->CheckNewData(data, &accuracy) && HAL_GetTick() - t >= 20000)
			printf("data: %f %f %f\n", data[0], data[1], data[2]);
	}
}
