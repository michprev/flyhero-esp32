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
	const float MAHONY_KP, MAHONY_KI;
	const float DELTA_T;
	IMU::Sensor_Data mahony_integral;

public:
	Mahony_Filter(float kp, float ki, uint16_t sample_rate);

	void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles& euler);
	void Reset();

};

} /* namespace flyhero */

#endif /* MAIN_MAHONY_FILTER_H_ */
