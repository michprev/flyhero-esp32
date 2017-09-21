#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <iostream>
#include "HC_SR04.h"

using namespace flyhero;

static void ultrasonic_handler(void *arg);

HC_SR04 us1(GPIO_NUM_32, GPIO_NUM_35, ultrasonic_handler);

extern "C" void app_main(void) {
	us1.Init();

	while (true) {
		us1.Trigger();

		vTaskDelay(60 / portTICK_RATE_MS);
		std::cout << us1.Get_Distance() << std::endl;
	}
}

static void ultrasonic_handler(void *arg) {
	switch ((uint32_t)arg) {
	case GPIO_NUM_35:
		us1.Echo_Callback();
		break;
	}
}
