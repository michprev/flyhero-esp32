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
			
extern "C" void initialise_monitor_handles(void);

Logger *logger = Logger::Instance();

extern "C" void HardFault_Handler(void)
{
	printf("hard fault\n");
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&logger->hdma_uart5_tx);
}

extern "C" void UART5_IRQHandler(void)
{
	HAL_UART_IRQHandler(&logger->huart);
}


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
