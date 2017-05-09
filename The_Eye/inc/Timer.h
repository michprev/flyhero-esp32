/*
 * Timer.h
 *
 *  Created on: 8. 5. 2017
 *      Author: michp
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stm32f4xx_hal.h>

namespace flyhero {

class Timer {
private:
	static TIM_HandleTypeDef htim5;

public:
	static TIM_HandleTypeDef* Get_Handle();
	static void Delay_ms(uint16_t ms);
	static void Delay_us(uint16_t us);
	static uint32_t Get_Tick_Count();
	//static void Start_Task(void (*func)(void), uint16_t period);
};

} /* namespace flyhero */

#endif /* TIMER_H_ */
