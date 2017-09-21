/*
 * LEDs.h
 *
 *  Created on: 17. 9. 2017
 *      Author: michp
 */

#pragma once

#include <driver/gpio.h>
#include <esp_err.h>

namespace flyhero {

class LEDs {
private:
	LEDs() {};

public:
	enum Color { ONBOARD = 1 << 0 };

	static void Init();
	static void Turn_On(Color color);
	static void Turn_Off(Color color);

};

inline LEDs::Color operator|(LEDs::Color a, LEDs::Color b) {
		return static_cast<LEDs::Color>(static_cast<int>(a) | static_cast<int>(b));
}

} /* namespace flyhero */
