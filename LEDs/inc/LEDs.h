/*
 * LEDs.h
 *
 *  Created on: 3. 1. 2017
 *      Author: michp
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "stm32f4xx_hal.h"

class LEDs {
public:
	static const uint8_t Green = 1 << 0;
	static const uint8_t Orange = 1 << 1;
	static const uint8_t Yellow = 1 << 2;

	static void Init();
	static void TurnOn(uint8_t color);
	static void TurnOff(uint8_t color);
	static void Toggle(uint8_t color);
};

#endif /* LEDS_H_ */
