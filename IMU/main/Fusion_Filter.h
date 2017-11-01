/*
 * Fusion_Filter.h
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#pragma once

#include "IMU.h"


namespace flyhero
{

class Fusion_Filter
{
public:
    virtual ~Fusion_Filter() = 0;

    virtual void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler) = 0;
    virtual void Reset() = 0;
};

inline Fusion_Filter::~Fusion_Filter()
{

}

} /* namespace flyhero */
