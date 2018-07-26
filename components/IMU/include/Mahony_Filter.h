/*
 * Mahony_Filter.h
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#pragma once

#include <cmath>
#include <esp_timer.h>

#include "Fusion_Filter.h"
#include "Math.h"


namespace flyhero
{

class Mahony_Filter : public Fusion_Filter
{
private:
    IMU::Quaternion quaternion;
    const float TWO_KP, TWO_KI;
    int64_t first_time, last_time;
    IMU::Sensor_Data error_integral;

public:
    Mahony_Filter(float kp, float ki);

    ~Mahony_Filter() override = default;

    void Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler) override;

    void Reset() override;
};

} /* namespace flyhero */
