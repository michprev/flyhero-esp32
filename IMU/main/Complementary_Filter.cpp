/*
 * Complementary_Filter.cpp
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#include "Complementary_Filter.h"


namespace flyhero
{

Complementary_Filter::Complementary_Filter(float coeff, uint16_t sample_rate)
        : COMPLEMENTARY_COEFFICIENT(coeff), DELTA_T(1.0f / sample_rate)
{
    this->euler.roll = 0;
    this->euler.pitch = 0;
    this->euler.yaw = 0;
}

void Complementary_Filter::Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler)
{
    float accel_roll = Math::Atan2(accel.y, accel.z);

    float accel_pitch = Math::Atan2(-accel.x, std::sqrt((float) (accel.y * accel.y + accel.z * accel.z)));

    this->euler.roll = this->COMPLEMENTARY_COEFFICIENT * (this->euler.roll + gyro.y * this->DELTA_T) +
                       (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_roll;

    this->euler.pitch = this->COMPLEMENTARY_COEFFICIENT * (this->euler.pitch + gyro.x * this->DELTA_T) +
                        (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_pitch;

    this->euler.yaw += gyro.z * this->DELTA_T;

    if (this->euler.yaw > 180)
        this->euler.yaw -= 360;
    else if (this->euler.yaw < -180)
        this->euler.yaw += 360;

    euler = this->euler;
}

void Complementary_Filter::Reset()
{
    this->euler.roll = 0;
    this->euler.pitch = 0;
    this->euler.yaw = 0;
}

} /* namespace flyhero */
