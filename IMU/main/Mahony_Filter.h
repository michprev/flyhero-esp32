/*
 * Mahony_Filter.h
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#pragma once

#include <cmath>

#include "Fusion_Filter.h"
#include "Math.h"


namespace flyhero
{

class Mahony_Filter : public Fusion_Filter
{
private:
    IMU::Quaternion quaternion;
    const float MAHONY_KP, MAHONY_KI;
    const float DELTA_T;
    IMU::Sensor_Data mahony_integral;

public:
    Mahony_Filter(float kp, float ki, uint16_t sample_rate);
    ~Mahony_Filter() override = default;

    void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler) override;
    void Reset() override;

};

} /* namespace flyhero */
