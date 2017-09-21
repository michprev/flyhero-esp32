/*
 * LEDs.cpp
 *
 *  Created on: 17. 9. 2017
 *      Author: michp
 */

#include "LEDs.h"

namespace flyhero {

void LEDs::Init() {
	gpio_config_t gpio_conf;
	gpio_conf.intr_type = GPIO_INTR_DISABLE;
	gpio_conf.mode = GPIO_MODE_OUTPUT;
	gpio_conf.pin_bit_mask = GPIO_SEL_2 | GPIO_SEL_33;
	gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;

	ESP_ERROR_CHECK(gpio_config(&gpio_conf));

	ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
	ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_33, 0));
}

void LEDs::Turn_On(Color color) {
	if (color & ONBOARD)
		ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
	if (color & WARNING)
		ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_33, 1));
}

void LEDs::Turn_Off(Color color) {
	if (color & ONBOARD)
		ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
	if (color & WARNING)
		ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_33, 0));
}

} /* namespace flyhero */
