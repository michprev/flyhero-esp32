/*
 * ESP32.h
 *
 *  Created on: 10. 3. 2017
 *      Author: michp
 */

#ifndef ESP32_H_
#define ESP32_H_

#include <stm32f4xx_hal.h>
#include "ESP.h"

namespace flyhero {

class ESP32 : ESP {
private:
	/* Singleton */
	ESP32();
	ESP32(ESP32 const&){};
	ESP32& operator=(ESP32 const&){};
	static ESP* pInstance;

public:
	// TODO should not be defined public
	static ESP* Instance();

	HAL_StatusTypeDef Init(void (*IPD_callback)(uint8_t linkID, uint8_t *data, uint16_t length)) override;
};

}

#endif /* ESP32_H_ */
