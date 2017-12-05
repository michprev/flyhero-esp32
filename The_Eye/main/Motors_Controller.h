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

enum Axis
{
    Roll, Pitch, Yaw
};

class Motors_Controller
{
private:
    Motors_Controller();
    Motors_Controller(Motors_Controller const &);
    Motors_Controller &operator=(Motors_Controller const &);

    PWM_Generator &pwm;
    PID *roll_PID, *pitch_PID, *yaw_PID;
    int16_t motor_FL, motor_FR, motor_BL, motor_BR;
    uint16_t throttle;
    bool invert_yaw;

    SemaphoreHandle_t roll_PID_semaphore, pitch_PID_semaphore, yaw_PID_semaphore,
            throttle_semaphore, invert_yaw_semaphore;

public:
    static Motors_Controller &Instance();

    // to be called from CORE 0
    void Init();
    void Set_PID_Constants(Axis axis, double Kp, double Ki, double Kd);
    void Set_Throttle(uint16_t throttle);
    void Set_Invert_Yaw(bool invert);

    // to be called from CORE 1
    void Update_Motors(IMU::Euler_Angles euler);
};

} /* namespace flyhero */
