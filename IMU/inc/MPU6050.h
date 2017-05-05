/*
 * MPU6050.h
 *
 *  Created on: 13. 12. 2016
 *      Author: michp
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stm32f4xx_hal.h>

extern "C" {

#include "invensense\inv_mpu.h"
#include "invensense\inv_mpu_dmp_motion_driver.h"
#include "invensense\mltypes.h"
#include "invensense\mpl.h"
#include "invensense\library\invensense_adv.h"
#include "invensense\eMPL_outputs.h"
#include "invensense\invensense.h"
#include "invensense\i2c.h"

}

namespace flyhero {

class MPU6050 {
private:
	MPU6050(){};
	MPU6050(MPU6050 const&){};
	MPU6050& operator=(MPU6050 const&){};
	static MPU6050* pInstance;

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
	struct Sensor_Data {
		double x, y, z;
	};

	volatile bool dataReady;

	static MPU6050* Instance();
	uint8_t Init(I2C_HandleTypeDef *hi2c);
	uint8_t SelfTest();
	uint8_t CheckNewData();
	uint8_t ReadGyro(Sensor_Data *data);
	uint8_t ReadAccel(Sensor_Data *data);
	uint8_t ReadEuler(Sensor_Data *data);
};

}

#endif /* MPU6050_H_ */
