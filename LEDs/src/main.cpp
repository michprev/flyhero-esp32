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
			
using namespace flyhero;

int main(void)
{
	HAL_Init();

	LEDs::Init();

	LEDs::TurnOn(LEDs::Green | LEDs::Yellow);

	while (true) {
		LEDs::Toggle(LEDs::Green | LEDs::Orange | LEDs::Yellow);

		HAL_Delay(500);
	}
}
