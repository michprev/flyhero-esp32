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
#include "stm32f4xx_hal.h"
#include "ArduCAM.h"
			
extern "C" void initialise_monitor_handles(void);

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	ArduCAM *camera = ArduCAM::Instance();

	camera->Init();

	HAL_Delay(1000);

	camera->Capture();

	HAL_Delay(1000);

	camera->Capture();

	while (true) {
		HAL_Delay(10);
	}
}
