/*
 * Mahony_Filter.cpp
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#include "Mahony_Filter.h"


namespace flyhero
{

Mahony_Filter::Mahony_Filter(float kp, float ki)
        : TWO_KP(2 * kp), TWO_KI(2 * ki)
{
    this->last_time = 0;
    this->quaternion.q0 = 1;
    this->quaternion.q1 = 0;
    this->quaternion.q2 = 0;
    this->quaternion.q3 = 0;
    this->error_integral.x = 0;
    this->error_integral.y = 0;
    this->error_integral.z = 0;
}

// expects gyro data in deg/s, accel data in multiples of g
void Mahony_Filter::Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler)
{
    float delta_t;

    if (this->last_time == 0)
    {
        delta_t = 0;

        this->last_time = esp_timer_get_time();
    } else
    {
        int64_t tmp = esp_timer_get_time();
        delta_t = (tmp - this->last_time) * 0.000001f;

        this->last_time = tmp;
    }

    float recip_norm;

    IMU::Sensor_Data gyro_rad;
    gyro_rad.x = gyro.x * Math::DEG_TO_RAD;
    gyro_rad.y = gyro.y * Math::DEG_TO_RAD;
    gyro_rad.z = gyro.z * Math::DEG_TO_RAD;

    IMU::Sensor_Data half_v, half_error;

    // normalise accelerometer measurement
    recip_norm = 1 / sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x *= recip_norm;
    accel.y *= recip_norm;
    accel.z *= recip_norm;

    // estimated direction of gravity and vector perpendicular to magnetic flux
    half_v.x = this->quaternion.q1 * this->quaternion.q3
               - this->quaternion.q0 * this->quaternion.q2;
    half_v.y = this->quaternion.q0 * this->quaternion.q1
               + this->quaternion.q2 * this->quaternion.q3;
    half_v.z = this->quaternion.q0 * this->quaternion.q0 - 0.5f
               + this->quaternion.q3 * this->quaternion.q3;

    // error is sum of cross product between estimated and measured direction of gravity
    half_error.x = accel.y * half_v.z - accel.z * half_v.y;
    half_error.y = accel.z * half_v.x - accel.x * half_v.z;
    half_error.z = accel.x * half_v.y - accel.y * half_v.x;

    if (this->TWO_KI > 0)
    {
        this->error_integral.x += this->TWO_KI * half_error.x * delta_t;
        this->error_integral.y += this->TWO_KI * half_error.y * delta_t;
        this->error_integral.z += this->TWO_KI * half_error.z * delta_t;
    } else
    {
        this->error_integral.x = 0;
        this->error_integral.y = 0;
        this->error_integral.z = 0;
    }

    gyro_rad.x += this->TWO_KP * half_error.x + error_integral.x;
    gyro_rad.y += this->TWO_KP * half_error.y + error_integral.y;
    gyro_rad.z += this->TWO_KP * half_error.z + error_integral.z;

    gyro_rad.x *= 0.5f * delta_t;
    gyro_rad.y *= 0.5f * delta_t;
    gyro_rad.z *= 0.5f * delta_t;

    float qa = this->quaternion.q0;
    float qb = this->quaternion.q1;
    float qc = this->quaternion.q2;

    this->quaternion.q0 += -qb * gyro_rad.x - qc * gyro_rad.y
                           - this->quaternion.q3 * gyro_rad.z;
    this->quaternion.q1 += qa * gyro_rad.x + qc * gyro_rad.z
                           - this->quaternion.q3 * gyro_rad.y;
    this->quaternion.q2 += qa * gyro_rad.y - qb * gyro_rad.z
                           + this->quaternion.q3 * gyro_rad.x;
    this->quaternion.q3 += qa * gyro_rad.z + qb * gyro_rad.y
                           - qc * gyro_rad.x;

    // normalise quaternion
    recip_norm = 1 / sqrtf(this->quaternion.q0 * this->quaternion.q0 +
                           this->quaternion.q1 * this->quaternion.q1 +
                           this->quaternion.q2 * this->quaternion.q2 +
                           this->quaternion.q3 * this->quaternion.q3);

    this->quaternion.q0 *= recip_norm;
    this->quaternion.q1 *= recip_norm;
    this->quaternion.q2 *= recip_norm;
    this->quaternion.q3 *= recip_norm;

    // convert quaternion to euler
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion

    float q2_sqr = this->quaternion.q2 * this->quaternion.q2;

    euler.roll = atan2f(2 * (this->quaternion.q0 * this->quaternion.q1 + this->quaternion.q2 * this->quaternion.q3),
                        1 - 2 * (this->quaternion.q1 * this->quaternion.q1 + q2_sqr)) * Math::RAD_TO_DEG;

    float sinp = 2 * (this->quaternion.q0 * this->quaternion.q2 - this->quaternion.q3 * this->quaternion.q1);

    if (fabs(sinp) >= 1)
        euler.pitch = copysignf(Math::PI / 2, sinp);
    else
        euler.pitch = asinf(sinp);
    euler.pitch *= Math::RAD_TO_DEG;

    euler.yaw = atan2f(2 * (this->quaternion.q0 * this->quaternion.q3 + this->quaternion.q1 * this->quaternion.q2),
                           1 - 2 * (q2_sqr + this->quaternion.q3 * this->quaternion.q3)) * Math::RAD_TO_DEG;
}

void Mahony_Filter::Reset()
{
    this->last_time = 0;

    this->quaternion.q0 = 1;
    this->quaternion.q1 = 0;
    this->quaternion.q2 = 0;
    this->quaternion.q3 = 0;

    this->error_integral.x = 0;
    this->error_integral.y = 0;
    this->error_integral.z = 0;
}

} /* namespace flyhero */
