/*
 * Complementary_Filter.h
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#pragma once

#include <sys/time.h>

#include "Fusion_Filter.h"
#include "Math.h"


namespace flyhero
{

class Complementary_Filter : public Fusion_Filter
{
private:
    IMU::Euler_Angles euler;
    timeval last_time;
    const double COMPLEMENTARY_COEFFICIENT;

public:
    Complementary_Filter(double coeff);
    ~Complementary_Filter() override = default;

    void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler) override;
    void Reset() override;
};

} /* namespace flyhero */
