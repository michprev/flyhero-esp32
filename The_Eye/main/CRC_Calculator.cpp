/*
 * CRC_Calculator.cpp
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#include <CRC_Calculator.h>

namespace flyhero {

CRC_Calculator& CRC_Calculator::Instance() {
	static CRC_Calculator instance;

	return instance;
}

HAL_StatusTypeDef CRC_Calculator::Init() {
	this->hcrc.Instance = CRC;

	return HAL_CRC_Init(&this->hcrc);
}

// reset CRC data register to 0xFFFF FFFF and calculate 32 b CRC
uint32_t CRC_Calculator::Calculate(uint32_t *buffer, uint32_t buffer_length) {
	return HAL_CRC_Calculate(&this->hcrc, buffer, buffer_length);
}

// calculate 32 b CRC with last CRC value
uint32_t CRC_Calculator::Accumulate(uint32_t *buffer, uint32_t buffer_length) {
	return HAL_CRC_Accumulate(&this->hcrc, buffer, buffer_length);
}

} /* namespace flyhero */
