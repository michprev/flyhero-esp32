/*
 * Timer.cpp
 *
 *  Created on: 8. 5. 2017
 *      Author: michp
 */

#include <Timer.h>

namespace flyhero {

extern "C" {
	void TIM5_IRQHandler(void)
	{
		TIM_HandleTypeDef *htim5 = Timer::Get_Handle();

		// Channel 2 for HAL 1 ms tick
		if (__HAL_TIM_GET_ITSTATUS(htim5, TIM_IT_CC2) == SET) {
			__HAL_TIM_CLEAR_IT(htim5, TIM_IT_CC2);
			uint32_t val = __HAL_TIM_GetCounter(htim5);
			//if ((val - prev) >= 1000) {
				HAL_IncTick();
				// Prepare next interrupt
				__HAL_TIM_SetCompare(htim5, TIM_CHANNEL_2, val + 1000);
				//prev = val;
			//}
			//else {
			//	printf("should not happen\n");
			//}
		}
		//HAL_TIM_IRQHandler(Timer::Get_Handle());
	}

	/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if (htim->Instance == TIM5) {
			HAL_IncTick();
		}
	}*/


	HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
	{
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		uint32_t PclkFreq;

		// Get clock configuration
		// Note: PclkFreq contains here the Latency (not used after)
		HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

		// Get TIM5 clock value
		PclkFreq = HAL_RCC_GetPCLK1Freq();

		// Enable timer clock
		__TIM5_CLK_ENABLE();

		// Reset timer
		__TIM5_FORCE_RESET();
		__TIM5_RELEASE_RESET();

		TIM_HandleTypeDef *htim5 = Timer::Get_Handle();

		// Configure time base
		htim5->Instance = TIM5;
		htim5->Init.Period = 0xFFFFFFFF;
		htim5->Init.Prescaler = (uint16_t)((PclkFreq) / 1000000) - 1; // 1 us tick
		htim5->Init.ClockDivision = RCC_HCLK_DIV1;
		htim5->Init.CounterMode = TIM_COUNTERMODE_UP;
		htim5->Init.RepetitionCounter = 0;
		if (HAL_TIM_OC_Init(htim5))
			return HAL_ERROR;

		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);

		// Channel 1 for 1 us tick with no interrupt
		if (HAL_TIM_OC_Start(htim5, TIM_CHANNEL_1))
			return HAL_ERROR;

		// Channel 2 for 1 us tick with interrupt every 1 ms
		if (HAL_TIM_OC_Start(htim5, TIM_CHANNEL_2))
			return HAL_ERROR;

		__HAL_TIM_SetCompare(htim5, TIM_CHANNEL_2, __HAL_TIM_GetCounter(htim5) + 1000);
		__HAL_TIM_ENABLE_IT(htim5, TIM_IT_CC2);
	}

	void HAL_ResumeTick(void)
	{
		HAL_TIM_OC_Start(Timer::Get_Handle(), TIM_CHANNEL_2);
		//__HAL_TIM_ENABLE_IT(Timer::Get_Handle(), TIM_IT_UPDATE);
	}

	void HAL_SuspendTick(void)
	{
		HAL_TIM_OC_Stop(Timer::Get_Handle(), TIM_CHANNEL_2);
		//__HAL_TIM_DISABLE_IT(Timer::Get_Handle(), TIM_IT_UPDATE);
	}

}

TIM_HandleTypeDef Timer::htim5 = TIM_HandleTypeDef();

TIM_HandleTypeDef* Timer::Get_Handle() {
	return &Timer::htim5;
}

void Timer::Delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

void Timer::Delay_us(uint16_t us) {
	uint32_t start = Timer::htim5.Instance->CNT;

	while (Timer::htim5.Instance->CNT - start < us);
}

uint32_t Timer::Get_Tick_Count() {
	return Timer::htim5.Instance->CNT;
}

} /* namespace flyhero */
