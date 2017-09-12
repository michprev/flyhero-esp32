#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "PWM_Generator.h"

using namespace flyhero;

extern "C" void app_main(void)
{
	PWM_Generator& pwm = PWM_Generator::Instance();

	pwm.Init();

	vTaskDelay(1000 / portTICK_RATE_MS);

	pwm.Arm();

	while (true) {
		pwm.Set_Pulse(PWM_Generator::MOTOR_FL, 100);
		pwm.Set_Pulse(PWM_Generator::MOTOR_BL, 100);
		pwm.Set_Pulse(PWM_Generator::MOTOR_FR, 100);
		pwm.Set_Pulse(PWM_Generator::MOTOR_BR, 100);

		vTaskDelay(1000 / portTICK_RATE_MS);

		pwm.Set_Pulse(PWM_Generator::MOTOR_FL, 0);
		pwm.Set_Pulse(PWM_Generator::MOTOR_BL, 0);
		pwm.Set_Pulse(PWM_Generator::MOTOR_FR, 0);
		pwm.Set_Pulse(PWM_Generator::MOTOR_BR, 0);

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
