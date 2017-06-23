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
#include "Logger.h"

using namespace flyhero;

extern "C" void initialise_monitor_handles(void);

Logger *logger = Logger::Instance();

int main(void)
{
	HAL_Init();

	logger->Init();

	while (true) {
		if (logger->Print((uint8_t*)"abc\n", 4) != HAL_OK)
			logger->Print((uint8_t*)"e", 1);

		HAL_Delay(5);
	}
}
