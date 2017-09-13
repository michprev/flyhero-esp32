/*
 * PWM_Generator.cpp
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#include "PWM_Generator.h"

namespace flyhero {

PWM_Generator& PWM_Generator::Instance() {
	static PWM_Generator instance;

	return instance;
}

void PWM_Generator::Init()
{
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_32);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_NUM_33);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_NUM_25);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_NUM_26);

	mcpwm_config_t pwm_config;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	pwm_config.frequency = 1000;

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
	mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}

void PWM_Generator::Set_Pulse(Motor_Type motor, uint16_t us)
{
	if (us > 1000)
		return;

	if (motor & MOTOR_FL)
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, us);
	if (motor & MOTOR_BL)
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, us);
	if (motor & MOTOR_FR)
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, us);
	if (motor & MOTOR_BR)
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, us);
}

void PWM_Generator::Arm()
{
	// we set maximum pulse here
	this->Set_Pulse(MOTORS_ALL, 1000);

	vTaskDelay(1000 / portTICK_RATE_MS);

	// we set minimum pulse here
	this->Set_Pulse(MOTORS_ALL, 0);
}

}
