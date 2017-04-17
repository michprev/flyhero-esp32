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
#include "ArduCAM_OV5642.h".h"
			
extern "C" void initialise_monitor_handles(void);

ArduCAM_OV5642 *camera = ArduCAM_OV5642::Instance();

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

	void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
		printf("aa");
	}
}

void SystemClockHSE_Config(void);

int main(void)
{
	HAL_Init();

	//SystemClockHSE_Config();

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

void SystemClockHSE_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* -1- Select HSI as system clock source to allow modification of the PLL configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    //Error_Handler();
  }

  /* -2- Enable HSE Oscillator, select it as PLL source and finally activate the PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
 /* HSE_CRYSTAL */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler();
  }

   /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    //Error_Handler();
  }

  /* -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler();
  }
}
