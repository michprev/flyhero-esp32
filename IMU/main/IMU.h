/*
 * IMU.h
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#ifndef MAIN_IMU_H_
#define MAIN_IMU_H_

#include <stdint.h>

namespace flyhero {

class IMU {
public:
	struct Raw_Data {
		int16_t x, y, z;
	};

	struct Sensor_Data {
		float x, y, z;
	};

	struct Euler_Angles {
		float roll, pitch, yaw;
	};

	struct Quaternion {
		float q0, q1, q2, q3;
	};

};

} /* namespace flyhero */

#endif /* MAIN_IMU_H_ */
