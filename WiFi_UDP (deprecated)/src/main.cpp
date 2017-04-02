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
#include "ESP8266_UDP.h"
			
extern "C" void initialise_monitor_handles(void);

ESP8266_UDP *esp8266 = ESP8266_UDP::Instance();
IWDG_HandleTypeDef hiwdg;

extern "C" void HardFault_Handler(void)
{
	printf("hard fault\n");
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&esp8266->hdma_usart3_tx);
}

extern "C" void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&esp8266->huart);
}

void IPD_Callback(uint8_t *data, uint16_t length) {
	printf("IPD: %s\n", data);
}

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	printf("init\n");

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 2047;

	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	esp8266->Reset();

	HAL_Delay(1000);

	if (esp8266->UART_Init() != HAL_OK) {
		while (true);
	}

	esp8266->IPD_Callback = &IPD_Callback;
	//esp8266->output = true;
	esp8266->Init();

	printf("init complete\n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		printf("Could not start watchdog\n");
	}

	while (true) {
		HAL_IWDG_Refresh(&hiwdg);
		esp8266->ProcessData();
	}
}
