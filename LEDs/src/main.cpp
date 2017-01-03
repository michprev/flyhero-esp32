/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "LEDs.h"
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal.h"
			

int main(void)
{
	HAL_Init();

	LEDs::Init();

	while (true) {
		LEDs::TurnOn(LEDs::Green | LEDs::Orange | LEDs::Yellow);

		HAL_Delay(500);

		LEDs::TurnOff(LEDs::Green | LEDs::Orange | LEDs::Yellow);

		HAL_Delay(500);
	}
}
