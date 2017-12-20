/*
 * Mahony_Filter.cpp
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#include "Mahony_Filter.h"


namespace flyhero
{

Mahony_Filter::Mahony_Filter(double kp, double ki)
        : MAHONY_KP(kp), MAHONY_KI(ki)
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
    double delta_t;

    if (this->last_time == 0)
    {
        delta_t = 0;

        this->last_time = esp_timer_get_time();
    }
    else
    {
        int64_t tmp = esp_timer_get_time();
        delta_t = (tmp - this->last_time) * 0.000001;

        this->last_time = tmp;
    }

    double recip_norm;

    IMU::Sensor_Data gyro_rad;
    gyro_rad.x = gyro.x * Math::DEG_TO_RAD;
    gyro_rad.y = gyro.y * Math::DEG_TO_RAD;
    gyro_rad.z = gyro.z * Math::DEG_TO_RAD;

    IMU::Sensor_Data v, error;

    // normalise accelerometer measurement
    recip_norm = 1 / std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
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

    double qa = this->quaternion.q0;
    double qb = this->quaternion.q1;
    double qc = this->quaternion.q2;

    this->quaternion.q0 += 0.5 * delta_t * (-qb * gyro_rad.x - qc * gyro_rad.y
                                                   - this->quaternion.q3 * gyro_rad.z);
    this->quaternion.q1 += 0.5 * delta_t * (qa * gyro_rad.x + qc * gyro_rad.z
                                                   - this->quaternion.q3 * gyro_rad.y);
    this->quaternion.q2 += 0.5 * delta_t * (qa * gyro_rad.y - qb * gyro_rad.z
                                                   + this->quaternion.q3 * gyro_rad.x);
    this->quaternion.q3 += 0.5 * delta_t * (qa * gyro_rad.z + qb * gyro_rad.y
                                                   - qc * gyro_rad.x);

    // normalise quaternion
    recip_norm = 1 / std::sqrt(this->quaternion.q0 * this->quaternion.q0 +
                                this->quaternion.q1 * this->quaternion.q1 +
                                this->quaternion.q2 * this->quaternion.q2 +
                                this->quaternion.q3 * this->quaternion.q3);

    this->quaternion.q0 *= recip_norm;
    this->quaternion.q1 *= recip_norm;
    this->quaternion.q2 *= recip_norm;
    this->quaternion.q3 *= recip_norm;

    // convert quaternion to euler
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion

    double q2_sqr = this->quaternion.q2 * this->quaternion.q2;

    double t0 = +2.0 * (this->quaternion.q0 * this->quaternion.q1 + this->quaternion.q2 * this->quaternion.q3);
    double t1 = +1.0 - 2.0 * (this->quaternion.q1 * this->quaternion.q1 + q2_sqr);
    euler.roll = std::atan2(t0, t1) * Math::RAD_TO_DEG;

    double t2 = +2.0 * (this->quaternion.q0 * this->quaternion.q2 - this->quaternion.q3 * this->quaternion.q1);
    t2 = ((t2 > 1.0) ? 1.0 : t2);
    t2 = ((t2 < -1.0) ? -1.0 : t2);
    euler.pitch = std::asin(t2);
    euler.pitch *= Math::RAD_TO_DEG;

    double t3 = +2.0 * (this->quaternion.q0 * this->quaternion.q3 + this->quaternion.q1 * this->quaternion.q2);
    double t4 = +1.0 - 2.0 * (q2_sqr + this->quaternion.q3 * this->quaternion.q3);
    euler.yaw = std::atan2(t3, t4) * Math::RAD_TO_DEG;
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
