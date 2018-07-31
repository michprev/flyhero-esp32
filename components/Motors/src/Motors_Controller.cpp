/*
 * Motors_Controller.cpp
 *
 *  Created on: 18. 7. 2017
 *      Author: michp
 */

#include "OneShot125.h"
#include "Motors_Controller.h"


namespace flyhero
{

Motors_Controller &Motors_Controller::Instance()
{
    static Motors_Controller instance;

    return instance;
}

Motors_Controller::Motors_Controller() : motors_protocol(OneShot125::Instance())
{
    this->motors_protocol.Disarm();

    this->motor_FR = 0;
    this->motor_FL = 0;
    this->motor_BR = 0;
    this->motor_BL = 0;

    this->rate_setpoints[0] = 0;
    this->rate_setpoints[1] = 0;
    this->rate_setpoints[2] = 0;

    this->throttle = 0;

    this->running = false;

    this->stab_PIDs_mutex = portMUX_INITIALIZER_UNLOCKED;
    this->rate_PIDs_mutex = portMUX_INITIALIZER_UNLOCKED;
}

void Motors_Controller::Init()
{
    float sample_rate = IMU_Detector::Detect_IMU().Get_Gyro_Sample_Rate();

    vPortCPUAcquireMutex(&this->stab_PIDs_mutex);
    vPortCPUAcquireMutex(&this->rate_PIDs_mutex);

    this->stab_PIDs = new PID *[3];
    this->rate_PIDs = new PID *[3];

    for (uint8_t i = ROLL; i <= YAW; i++)
    {
        this->stab_PIDs[i] = new PID(sample_rate);
        this->stab_PIDs[i]->Set_I_Max(50);

        this->rate_PIDs[i] = new PID(sample_rate);
        this->rate_PIDs[i]->Set_I_Max(50);
    }

    this->stab_PIDs[ROLL]->Set_Kp(4.5f);
    this->stab_PIDs[PITCH]->Set_Kp(4.5f);

    vPortCPUReleaseMutex(&this->stab_PIDs_mutex);
    vPortCPUReleaseMutex(&this->rate_PIDs_mutex);

    this->motors_protocol.Init();
}

void Motors_Controller::Set_PID_Constants(PID_Type type, float parameters[3][3])
{
    portDISABLE_INTERRUPTS();

    switch (type)
    {
        case STABILIZE:
            vPortCPUAcquireMutex(&this->stab_PIDs_mutex);

            for (uint8_t i = ROLL; i <= YAW; i++)
            {
                this->stab_PIDs[i]->Set_Kp(parameters[i][0]);
                this->stab_PIDs[i]->Set_Ki(parameters[i][1]);
                this->stab_PIDs[i]->Set_Kd(parameters[i][2]);
            }

            vPortCPUReleaseMutex(&this->stab_PIDs_mutex);
            break;
        case RATE:
            vPortCPUAcquireMutex(&this->rate_PIDs_mutex);

            for (uint8_t i = ROLL; i <= YAW; i++)
            {
                this->rate_PIDs[i]->Set_Kp(parameters[i][0]);
                this->rate_PIDs[i]->Set_Ki(parameters[i][1]);
                this->rate_PIDs[i]->Set_Kd(parameters[i][2]);
            }

            vPortCPUReleaseMutex(&this->rate_PIDs_mutex);
            break;
    }

    portENABLE_INTERRUPTS();
}

void Motors_Controller::Set_Throttle(uint16_t throttle)
{
    if (throttle > 1000)
        return;

    this->throttle = throttle;
}

void Motors_Controller::Feed_Stab_PIDs(IMU::Euler_Angles euler)
{
    if (!this->running)
        return;

    // consider more than 60 deg unsafe
    if (std::fabs(euler.roll) > 60 || std::fabs(euler.pitch) > 60)
        esp_restart();

    uint16_t throttle = this->throttle;

    if (throttle > 180)
    {
        vPortCPUAcquireMutex(&this->stab_PIDs_mutex);

        this->rate_setpoints[ROLL] = this->stab_PIDs[ROLL]->Get_PID(0 - euler.roll);
        this->rate_setpoints[PITCH] = this->stab_PIDs[PITCH]->Get_PID(0 - euler.pitch);
        this->rate_setpoints[YAW] = 0;

        vPortCPUReleaseMutex(&this->stab_PIDs_mutex);
    }
}

void Motors_Controller::Feed_Rate_PIDs(IMU::Sensor_Data gyro)
{
    if (!this->running)
        return;

    uint16_t throttle = this->throttle;

    if (throttle > 180)
    {
        float rate_corrections[3];

        vPortCPUAcquireMutex(&this->rate_PIDs_mutex);

        rate_corrections[ROLL] =
                this->rate_PIDs[ROLL]->Get_PID(this->rate_setpoints[ROLL] - gyro.y);
        rate_corrections[PITCH] =
                this->rate_PIDs[PITCH]->Get_PID(this->rate_setpoints[PITCH] - gyro.x);
        rate_corrections[YAW] =
                this->rate_PIDs[YAW]->Get_PID(this->rate_setpoints[YAW] - gyro.z);

        vPortCPUReleaseMutex(&this->rate_PIDs_mutex);

        this->motor_FL = throttle - rate_corrections[ROLL] - rate_corrections[PITCH]
                         + rate_corrections[YAW];
        this->motor_BL = throttle - rate_corrections[ROLL] + rate_corrections[PITCH]
                         - rate_corrections[YAW];
        this->motor_FR = throttle + rate_corrections[ROLL] - rate_corrections[PITCH]
                         - rate_corrections[YAW];
        this->motor_BR = throttle + rate_corrections[ROLL] + rate_corrections[PITCH]
                         + rate_corrections[YAW];

        if (this->motor_FL > 1000)
            this->motor_FL = 1000;
        else if (this->motor_FL < 0)
            this->motor_FL = 0;

        if (this->motor_BL > 1000)
            this->motor_BL = 1000;
        else if (this->motor_BL < 0)
            this->motor_BL = 0;

        if (this->motor_FR > 1000)
            this->motor_FR = 1000;
        else if (this->motor_FR < 0)
            this->motor_FR = 0;

        if (this->motor_BR > 1000)
            this->motor_BR = 1000;
        else if (this->motor_BR < 0)
            this->motor_BR = 0;

        this->motors_protocol.Update(this->motor_FL, this->motor_BL, this->motor_FR, this->motor_BR);
    } else
        this->motors_protocol.Update(0, 0, 0, 0);
}

void Motors_Controller::Start()
{
    this->motors_protocol.Arm();
    this->running = true;
}

void Motors_Controller::Stop()
{
    this->motors_protocol.Disarm();
    this->running = false;
}

} /* namespace flyhero */
