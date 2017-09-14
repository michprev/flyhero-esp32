/*
 * CRC_Calculator.h
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#ifndef CRC_CALCULATOR_H_
#define CRC_CALCULATOR_H_

#include "stm32f4xx_hal.h"

namespace flyhero {

class CRC_Calculator {
private:
	CRC_Calculator(){};
	CRC_Calculator(CRC_Calculator const&){};
	CRC_Calculator& operator=(CRC_Calculator const&){};

	CRC_HandleTypeDef hcrc;

public:
	static CRC_Calculator& Instance();

	HAL_StatusTypeDef Init();
	uint32_t Calculate(uint32_t *buffer, uint32_t buffer_length);
	uint32_t Accumulate(uint32_t *buffer, uint32_t buffer_length);
};

} /* namespace flyhero */

#endif /* CRC_CALCULATOR_H_ */
