/*
 * Motors_Controller.h
 *
 *  Created on: 18. 7. 2017
 *      Author: michp
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "PID.h"
#include "IMU.h"
#include "PWM_Generator.h"
#include "IMU_Detector.h"


namespace flyhero
{

class Motors_Controller
{
private:
    Motors_Controller();

    Motors_Controller(Motors_Controller const &);

    Motors_Controller &operator=(Motors_Controller const &);

    enum axis
    {
        ROLL = 0,
        PITCH = 1,
        YAW = 2
    };

    PWM_Generator &pwm;
    PID **stab_PIDs, **rate_PIDs;
    Biquad_Filter *roll_filter, *pitch_filter, *yaw_filter;
    int16_t motor_FL, motor_FR, motor_BL, motor_BR;
    uint16_t throttle;
    bool invert_yaw;
    double reference_yaw;

    SemaphoreHandle_t stab_PIDs_semaphore, rate_PIDs_semaphore,
            throttle_semaphore, invert_yaw_semaphore;

public:
    static Motors_Controller &Instance();

    enum PID_Type
    {
        STABILIZE, RATE
    };

    // to be called from CORE 0
    void Init();

    void Set_PID_Constants(PID_Type type, double parameters[3][3]);

    void Set_Throttle(uint16_t throttle);

    void Set_Invert_Yaw(bool invert);

    // to be called from CORE 1
    void Update_Motors(IMU::Euler_Angles euler, IMU::Sensor_Data gyro);
};

} /* namespace flyhero */
