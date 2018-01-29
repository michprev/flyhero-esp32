/*
 * IMU.h
 *
 *  Created on: 10. 9. 2017
 *      Author: michp
 */

#pragma once

#include <cstdint>


namespace flyhero
{

class IMU
{
public:
    struct __attribute__((__packed__)) Raw_Data
    {
        int16_t x, y, z;
    };

    struct __attribute__((__packed__)) Sensor_Data
    {
        double x, y, z;
    };

    struct __attribute__((__packed__)) Euler_Angles
    {
        double roll, pitch, yaw;
    };

    struct __attribute__((__packed__)) Quaternion
    {
        double q0, q1, q2, q3;
    };

    virtual void Init() = 0;

    virtual bool Start() = 0;

    virtual void Accel_Calibrate() = 0;

    virtual void Gyro_Calibrate() = 0;

    virtual uint16_t Get_Sample_Rate() = 0;

    virtual void Read_Raw(Raw_Data &raw_accel, Raw_Data &raw_gyro) = 0;

    virtual void Read_Data(Sensor_Data &accel, Sensor_Data &gyro) = 0;

    virtual void Data_Ready_Callback() = 0;

    virtual bool Data_Ready() = 0;
};

} /* namespace flyhero */
