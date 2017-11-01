/*
 * Mahony_Filter.cpp
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#include "Mahony_Filter.h"


namespace flyhero
{

Mahony_Filter::Mahony_Filter(float kp, float ki, uint16_t sample_rate)
        : MAHONY_KP(kp), MAHONY_KI(ki), DELTA_T(1.0f / sample_rate)
{
    this->quaternion.q0 = 1;
    this->quaternion.q1 = 0;
    this->quaternion.q2 = 0;
    this->quaternion.q3 = 0;
    this->mahony_integral.x = 0;
    this->mahony_integral.y = 0;
    this->mahony_integral.z = 0;
}

// expects gyro data in deg/s, accel data in multiples of g
void Mahony_Filter::Compute(IMU::Sensor_Data accel, IMU::Sensor_Data gyro, IMU::Euler_Angles &euler)
{
    float recip_norm;

    IMU::Sensor_Data gyro_rad;
    gyro_rad.x = gyro.x * Math::DEG_TO_RAD;
    gyro_rad.y = gyro.y * Math::DEG_TO_RAD;
    gyro_rad.z = gyro.z * Math::DEG_TO_RAD;

    IMU::Sensor_Data half_v, half_e;

    // normalise accelerometer measurement
    recip_norm = Math::Inv_Sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x *= recip_norm;
    accel.y *= recip_norm;
    accel.z *= recip_norm;

    // estimated direction of gravity and vector perpendicular to magnetic flux
    half_v.x = this->quaternion.q1 * this->quaternion.q3 - this->quaternion.q0 * this->quaternion.q2;
    half_v.y = this->quaternion.q0 * this->quaternion.q1 + this->quaternion.q2 * this->quaternion.q3;
    half_v.z = this->quaternion.q0 * this->quaternion.q0 - 0.5f + this->quaternion.q3 * this->quaternion.q3;

    // error is sum of cross product between estimated and measured direction of gravity
    half_e.x = (accel.y * half_v.z - accel.z * half_v.y);
    half_e.y = (accel.z * half_v.x - accel.x * half_v.z);
    half_e.z = (accel.x * half_v.y - accel.y * half_v.x);

    if (this->MAHONY_KI > 0)
    {
        this->mahony_integral.x += 2 * this->MAHONY_KI * half_e.x * this->DELTA_T;
        this->mahony_integral.y += 2 * this->MAHONY_KI * half_e.y * this->DELTA_T;
        this->mahony_integral.z += 2 * this->MAHONY_KI * half_e.z * this->DELTA_T;

        gyro_rad.x += this->mahony_integral.x;
        gyro_rad.y += this->mahony_integral.y;
        gyro_rad.z += this->mahony_integral.z;
    }

    gyro_rad.x += 2 * this->MAHONY_KP * half_e.x;
    gyro_rad.y += 2 * this->MAHONY_KP * half_e.y;
    gyro_rad.z += 2 * this->MAHONY_KP * half_e.z;

    // integrate rate of change of quaternion
    gyro_rad.x *= 0.5f * this->DELTA_T;        // pre-multiply common factors
    gyro_rad.y *= 0.5f * this->DELTA_T;
    gyro_rad.z *= 0.5f * this->DELTA_T;

    float qa = this->quaternion.q0;
    float qb = this->quaternion.q1;
    float qc = this->quaternion.q2;

    this->quaternion.q0 += (-qb * gyro_rad.x - qc * gyro_rad.y - this->quaternion.q3 * gyro_rad.z);
    this->quaternion.q1 += (qa * gyro_rad.x + qc * gyro_rad.z - this->quaternion.q3 * gyro_rad.y);
    this->quaternion.q2 += (qa * gyro_rad.y - qb * gyro_rad.z + this->quaternion.q3 * gyro_rad.x);
    this->quaternion.q3 += (qa * gyro_rad.z + qb * gyro_rad.y - qc * gyro_rad.x);

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
    this->quaternion.q0 = 1;
    this->quaternion.q1 = 0;
    this->quaternion.q2 = 0;
    this->quaternion.q3 = 0;

    this->mahony_integral.x = 0;
    this->mahony_integral.y = 0;
    this->mahony_integral.z = 0;
}

} /* namespace flyhero */
