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
#include "NEO_M8N.h"

using namespace flyhero;

extern "C" void initialise_monitor_handles(void);

NEO_M8N *neo = NEO_M8N::Instance();

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&neo->hdma_usart1_rx);

	/*if (neo->hdma_usart1_rx.ErrorCode != 0) {
		printf("e");
	}*/
}

extern "C" void HardFault_Handler(void)
{
	printf("hard fault\n");
}

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	neo->Init();

	printf("GPS initialized\n");

	while (true) {
		neo->ParseData();
	}
}
