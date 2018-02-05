/*
 * Complementary_Filter.h
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#pragma once

#include <esp_timer.h>

#include "Fusion_Filter.h"
#include "Math.h"


namespace flyhero
{

class Complementary_Filter : public Fusion_Filter
{
private:
    IMU::Euler_Angles euler;
    int64_t last_time;
    const float COMPLEMENTARY_COEFFICIENT;

public:
    Complementary_Filter(float coeff);

    ~Complementary_Filter() override = default;

    void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler) override;

    void Reset() override;
};

} /* namespace flyhero */
