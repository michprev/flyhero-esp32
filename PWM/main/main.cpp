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
#include "PWM_Generator.h"

using namespace flyhero;

extern "C" void initialise_monitor_handles(void);

int main(void)
{
	HAL_Init();
	//initialise_monitor_handles();

	PWM_Generator& pwm = PWM_Generator::Instance();

	pwm.Init();
	pwm.Arm(NULL);

	pwm.SetPulse(1050, 1);
	pwm.SetPulse(1050, 2);
	pwm.SetPulse(1050, 3);
	pwm.SetPulse(1050, 4);

	for(;;);
}
