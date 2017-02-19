/*
 * MPU9250.h
 *
 *  Created on: 13. 12. 2016
 *      Author: michp
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include <stm32f4xx_hal.h>

extern "C" {

#include "invensense\inv_mpu.h"
#include "invensense\inv_mpu_dmp_motion_driver.h"
#include "invensense\mltypes.h"
#include "invensense\mpl.h"
#include "invensense\library\invensense_adv.h"
#include "invensense\eMPL_outputs.h"
#include "invensense\invensense.h"
#include "invensense\spi.h"

}

class MPU9250 {
private:
	MPU9250(){};
	MPU9250(MPU9250 const&){};
	MPU9250& operator=(MPU9250 const&){};
	static MPU9250* pInstance;

	struct platform_data_s {
		signed char orientation[9];
	};
	struct platform_data_s gyro_pdata = {
		{ 1, 0, 0,
		0, 1, 0,
		0, 0, 1 }
	};
	struct platform_data_s compass_pdata = {
		{ 0, 1, 0,
		1, 0, 0,
		0, 0, -1 }
	};
	const uint8_t COMPASS_READ_MS = 100;
	unsigned long next_temp_ms = 0;
	unsigned long next_compass_ms = 0;
	bool new_temp = false;
	unsigned long timestamp;
	const uint16_t GYRO_SAMPLE_RATE = 200; // Hz

	void IT_Init();

public:
	volatile bool dataReady;

	static MPU9250* Instance();
	uint8_t Init();
	uint8_t SelfTest();
	uint8_t CheckNewData(float *euler, uint8_t *accur);
};

#endif /* MPU9250_H_ */
