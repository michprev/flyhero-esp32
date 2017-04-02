/*
 * ESP8266.h
 *
 *  Created on: 2. 4. 2017
 *      Author: michp
 */

#ifndef ESP8266_H_
#define ESP8266_H_

#include <stm32f4xx_hal.h>
#include "ESP.h"

namespace The_Eye {

class ESP8266 : ESP {
private:
	/* Singleton */
	ESP8266();
	ESP8266(ESP8266 const&){};
	ESP8266& operator=(ESP8266 const&){};
	static ESP8266* pInstance;

public:
	static ESP* Instance();
	HAL_StatusTypeDef Init() override;
};

} /* namespace The_Eye */

#endif /* ESP8266_H_ */
