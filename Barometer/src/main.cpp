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
#include "MS5611.h"

extern "C" void initialise_monitor_handles(void);

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	MS5611 *ms5611 = MS5611::Instance();
	ms5611->Init();

	int32_t press, temp;

	ms5611->ConvertD1();

	while (true) {
		if (ms5611->D1_Ready())
			ms5611->ConvertD2();
		else if (ms5611->D2_Ready()) {
			ms5611->GetData(&temp, &press);
			printf("Temp: %d, press: %d\n", temp, press);

			HAL_Delay(100);
			ms5611->ConvertD1();
		}
	}
}
