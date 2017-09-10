/*
 * Mahony_Filter.h
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#ifndef MAIN_MAHONY_FILTER_H_
#define MAIN_MAHONY_FILTER_H_

#include <cmath>
#include "IMU.h"
#include "Math.h"

namespace flyhero {

class Mahony_Filter {
private:
	IMU::Quaternion quaternion;
	float mahony_Kp, mahony_Ki;
	IMU::Sensor_Data mahony_integral;

public:
	Mahony_Filter(float kp, float ki);

	void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles& euler);
	void Reset();

};

} /* namespace flyhero */

#endif /* MAIN_MAHONY_FILTER_H_ */
