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

ArduCAM *camera = ArduCAM::Instance();

extern "C" {
	void DMA2_Stream0_IRQHandler(void)
	{
	  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

	  /* USER CODE END DMA2_Stream0_IRQn 0 */
	  HAL_DMA_IRQHandler(&camera->hdma_spi1_rx);
	  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

	  /* USER CODE END DMA2_Stream0_IRQn 1 */
	}

	/**
	* @brief This function handles DMA2 stream3 global interrupt.
	*/
	void DMA2_Stream3_IRQHandler(void)
	{
	  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

	  /* USER CODE END DMA2_Stream3_IRQn 0 */
	  HAL_DMA_IRQHandler(&camera->hdma_spi1_tx);
	  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

	  /* USER CODE END DMA2_Stream3_IRQn 1 */
	}

	void SPI1_IRQHandler(void)
	{
	  /* USER CODE BEGIN SPI1_IRQn 0 */

	  /* USER CODE END SPI1_IRQn 0 */
	  HAL_SPI_IRQHandler(&camera->hspi);
	  /* USER CODE BEGIN SPI1_IRQn 1 */

	  /* USER CODE END SPI1_IRQn 1 */
	}

	void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
		printf("aaaaa\n");
	}
}

int main(void)
{
	HAL_Init();
	initialise_monitor_handles();

	//ArduCAM_OV5642 *camera = ArduCAM_OV5642::Instance();

	camera->Init();

	HAL_Delay(1000);

	//HAL_Delay(1000);

	//camera->Capture();

	uint32_t start;

	while (true) {
		start = HAL_GetTick();

		camera->Capture();

		//printf("%d\n", HAL_GetTick() - start);
	}
}
