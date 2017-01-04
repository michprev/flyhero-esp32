/*
 * MS5611.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#ifndef MS5611_H_
#define MS5611_H_

#include <stm32f4xx_hal.h>
#include <math.h>

class MS5611
{
public:
	static MS5611* Instance();
	HAL_StatusTypeDef Reset();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef GetData(int32_t *temperature, int32_t *pressure);
	bool D1_Ready();
	bool D2_Ready();
	HAL_StatusTypeDef ConvertD1();
	HAL_StatusTypeDef ConvertD2();
private:
	MS5611();
	MS5611(MS5611 const&){};
	MS5611& operator=(MS5611 const&){};
	static MS5611* pInstance;

	const uint8_t ADDRESS = 0xEE;
	uint32_t D1_Timestamp;
	uint32_t D2_Timestamp;
	uint32_t D1;
	uint32_t D2;
	uint16_t C[6];
	I2C_HandleTypeDef hi2c;

	HAL_StatusTypeDef I2C_Init();
};

#endif
