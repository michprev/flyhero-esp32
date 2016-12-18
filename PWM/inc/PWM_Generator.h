/*
 * PWM_Generator.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#ifndef PWM_GENERATOR_H_
#define PWM_GENERATOR_H_

#include <stm32f4xx_hal.h>

class PWM_Generator
{
private:
	TIM_HandleTypeDef htim;;
public:
	void Init();
	void SetPulse(uint16_t us, uint8_t index);
	void Arm();
};

#endif /* PWM_GENERATOR_H_ */
