/*
 * LEDs.h
 *
 *  Created on: 3. 1. 2017
 *      Author: michp
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "stm32f4xx_hal.h"

namespace flyhero {

class LEDs {
public:
	enum Color { Green = 1 << 0, Orange = 1 << 1, Yellow = 1 << 2 };

	static void Init();
	static void TurnOn(Color color);
	static void TurnOff(Color color);
	static void Toggle(Color color);
};

inline LEDs::Color operator|(LEDs::Color a, LEDs::Color b) {
		return static_cast<LEDs::Color>(static_cast<int>(a) | static_cast<int>(b));
}

}

#endif /* LEDS_H_ */
