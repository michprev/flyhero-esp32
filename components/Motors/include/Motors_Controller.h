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
#include "Motors_Protocol.h"
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

    Motors_Protocol &motors_protocol;
    PID **stab_PIDs, **rate_PIDs;
    int16_t motor_FL, motor_FR, motor_BL, motor_BR;
    uint16_t throttle;
    float rate_setpoints[3];
    bool running;

    SemaphoreHandle_t stab_PIDs_semaphore, rate_PIDs_semaphore;

public:
    static Motors_Controller &Instance();

    enum PID_Type
    {
        STABILIZE, RATE
    };

    // to be called from CORE 0
    void Init();

    void Set_PID_Constants(PID_Type type, float parameters[3][3]);

    void Set_Throttle(uint16_t throttle);

    // to be called from CORE 1
    void Start();

    void Stop();

    void Feed_Stab_PIDs(IMU::Euler_Angles euler);
    void Feed_Rate_PIDs(IMU::Sensor_Data gyro);
};

} /* namespace flyhero */
