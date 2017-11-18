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
        : MAHONY_KP(kp), MAHONY_KI(ki)
{
    this->last_time.tv_sec = 0;
    this->last_time.tv_usec = 0;
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

    float recip_norm;

    IMU::Sensor_Data gyro_rad;
    gyro_rad.x = gyro.x * Math::DEG_TO_RAD;
    gyro_rad.y = gyro.y * Math::DEG_TO_RAD;
    gyro_rad.z = gyro.z * Math::DEG_TO_RAD;

    IMU::Sensor_Data v, error;

    // normalise accelerometer measurement
    recip_norm = Math::Inv_Sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x *= recip_norm;
    accel.y *= recip_norm;
    accel.z *= recip_norm;

    // estimated direction of gravity and vector perpendicular to magnetic flux
    v.x = 2 * (this->quaternion.q1 * this->quaternion.q3
             - this->quaternion.q0 * this->quaternion.q2);
    v.y = 2 * (this->quaternion.q0 * this->quaternion.q1
             + this->quaternion.q2 * this->quaternion.q3);
    v.z = + this->quaternion.q0 * this->quaternion.q0
          - this->quaternion.q1 * this->quaternion.q1
          - this->quaternion.q2 * this->quaternion.q2
          + this->quaternion.q3 * this->quaternion.q3;

    // error is sum of cross product between estimated and measured direction of gravity
    error.x = (accel.y * v.z - accel.z * v.y);
    error.y = (accel.z * v.x - accel.x * v.z);
    error.z = (accel.x * v.y - accel.y * v.x);

    if (this->MAHONY_KI > 0)
    {
        this->error_integral.x += error.x * delta_t;
        this->error_integral.y += error.y * delta_t;
        this->error_integral.z += error.z * delta_t;
    } else
    {
        this->error_integral.x = 0;
        this->error_integral.y = 0;
        this->error_integral.z = 0;
    }

    gyro_rad.x += this->MAHONY_KP * error.x + this->MAHONY_KI * error_integral.x;
    gyro_rad.y += this->MAHONY_KP * error.y + this->MAHONY_KI * error_integral.y;
    gyro_rad.z += this->MAHONY_KP * error.z + this->MAHONY_KI * error_integral.z;

    float qa = this->quaternion.q0;
    float qb = this->quaternion.q1;
    float qc = this->quaternion.q2;

    this->quaternion.q0 += 0.5f * delta_t * (-qb * gyro_rad.x - qc * gyro_rad.y
                                                   - this->quaternion.q3 * gyro_rad.z);
    this->quaternion.q1 += 0.5f * delta_t * (qa * gyro_rad.x + qc * gyro_rad.z
                                                   - this->quaternion.q3 * gyro_rad.y);
    this->quaternion.q2 += 0.5f * delta_t * (qa * gyro_rad.y - qb * gyro_rad.z
                                                   + this->quaternion.q3 * gyro_rad.x);
    this->quaternion.q3 += 0.5f * delta_t * (qa * gyro_rad.z + qb * gyro_rad.y
                                                   - qc * gyro_rad.x);

    // normalise quaternion
    recip_norm = Math::Inv_Sqrt(this->quaternion.q0 * this->quaternion.q0 +
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

    float t0 = +2.0f * (this->quaternion.q0 * this->quaternion.q1 + this->quaternion.q2 * this->quaternion.q3);
    float t1 = +1.0f - 2.0f * (this->quaternion.q1 * this->quaternion.q1 + q2_sqr);
    euler.roll = Math::Atan2(t0, t1);

    float t2 = +2.0f * (this->quaternion.q0 * this->quaternion.q2 - this->quaternion.q3 * this->quaternion.q1);
    t2 = ((t2 > 1.0f) ? 1.0f : t2);
    t2 = ((t2 < -1.0f) ? -1.0f : t2);
    euler.pitch = std::asin(t2);
    euler.pitch *= Math::RAD_TO_DEG;

    float t3 = +2.0f * (this->quaternion.q0 * this->quaternion.q3 + this->quaternion.q1 * this->quaternion.q2);
    float t4 = +1.0f - 2.0f * (q2_sqr + this->quaternion.q3 * this->quaternion.q3);
    euler.yaw = Math::Atan2(t3, t4);
}

void Mahony_Filter::Reset()
{
    this->last_time.tv_sec = 0;
    this->last_time.tv_usec = 0;

    this->quaternion.q0 = 1;
    this->quaternion.q1 = 0;
    this->quaternion.q2 = 0;
    this->quaternion.q3 = 0;

    this->error_integral.x = 0;
    this->error_integral.y = 0;
    this->error_integral.z = 0;
}

} /* namespace flyhero */
