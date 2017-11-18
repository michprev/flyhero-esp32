/*
 * Complementary_Filter.cpp
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#include "Complementary_Filter.h"


namespace flyhero
{

Complementary_Filter::Complementary_Filter(float coeff)
        : COMPLEMENTARY_COEFFICIENT(coeff)
{
    this->last_time.tv_sec = 0;
    this->last_time.tv_usec = 0;
    this->euler.roll = 0;
    this->euler.pitch = 0;
    this->euler.yaw = 0;
}

void Complementary_Filter::Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler)
{
    float delta_t;

    if (this->last_time.tv_sec == 0 && this->last_time.tv_usec == 0)
    {
        delta_t = 0;

        gettimeofday(&this->last_time, NULL);
    }
    else
    {
        timeval tmp;

        gettimeofday(&tmp, NULL);

        delta_t = (tmp.tv_usec > this->last_time.tv_usec ?
                   tmp.tv_usec - this->last_time.tv_usec :
                   1000000 - tmp.tv_usec + this->last_time.tv_usec);
        delta_t *= 0.000001f;

        this->last_time = tmp;
    }

    float accel_roll = Math::Atan2(accel.y, accel.z);

    float accel_pitch =
            Math::Atan2(-accel.x, std::sqrt((float) (accel.y * accel.y + accel.z * accel.z)));

    this->euler.roll = this->COMPLEMENTARY_COEFFICIENT * (this->euler.roll + gyro.y * delta_t)
                       + (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_roll;

    this->euler.pitch = this->COMPLEMENTARY_COEFFICIENT * (this->euler.pitch + gyro.x * delta_t)
                        + (1 - this->COMPLEMENTARY_COEFFICIENT) * accel_pitch;

    this->euler.yaw += gyro.z * delta_t;

    if (this->euler.yaw > 180)
        this->euler.yaw -= 360;
    else if (this->euler.yaw < -180)
        this->euler.yaw += 360;

    euler = this->euler;
}

void Complementary_Filter::Reset()
{
    this->last_time.tv_sec = 0;
    this->last_time.tv_usec = 0;

    this->euler.roll = 0;
    this->euler.pitch = 0;
    this->euler.yaw = 0;
}

} /* namespace flyhero */
