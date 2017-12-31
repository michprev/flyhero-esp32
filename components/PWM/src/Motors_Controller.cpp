/*
 * Motors_Controller.cpp
 *
 *  Created on: 18. 7. 2017
 *      Author: michp
 */

#include "Motors_Controller.h"


namespace flyhero
{

Motors_Controller &Motors_Controller::Instance()
{
    static Motors_Controller instance;

    return instance;
}

Motors_Controller::Motors_Controller() : pwm(PWM_Generator::Instance())
{
    this->motor_FR = 0;
    this->motor_FL = 0;
    this->motor_BR = 0;
    this->motor_BL = 0;

    this->invert_yaw = true;
    this->throttle = 0;
    this->reference_yaw = 0;

    this->stab_PIDs_semaphore = xSemaphoreCreateBinary();
    this->rate_PIDs_semaphore = xSemaphoreCreateBinary();
    this->throttle_semaphore = xSemaphoreCreateBinary();
    this->invert_yaw_semaphore = xSemaphoreCreateBinary();
}

void Motors_Controller::Init()
{
    double sample_rate = IMU_Detector::Detect_IMU().Get_Sample_Rate();

    this->stab_PIDs = new PID *[3];
    this->rate_PIDs = new PID *[3];
    this->roll_filter = new Biquad_Filter(Biquad_Filter::FILTER_LOW_PASS, sample_rate, 100);
    this->pitch_filter = new Biquad_Filter(Biquad_Filter::FILTER_LOW_PASS, sample_rate, 100);
    this->yaw_filter = new Biquad_Filter(Biquad_Filter::FILTER_LOW_PASS, sample_rate, 100);

    for (uint8_t i = ROLL; i <= YAW; i++)
    {
        this->stab_PIDs[i] = new PID(sample_rate);
        this->stab_PIDs[i]->Set_I_Max(50);

        this->rate_PIDs[i] = new PID(sample_rate);
        this->rate_PIDs[i]->Set_I_Max(50);
    }

    this->stab_PIDs[ROLL]->Set_Kp(4.5);
    this->stab_PIDs[PITCH]->Set_Kp(4.5);

    xSemaphoreGive(this->stab_PIDs_semaphore);
    xSemaphoreGive(this->rate_PIDs_semaphore);
    xSemaphoreGive(this->throttle_semaphore);
    xSemaphoreGive(this->invert_yaw_semaphore);

    this->pwm.Init();
    this->pwm.Arm();
}

void Motors_Controller::Set_PID_Constants(PID_Type type, double parameters[3][3])
{
    switch (type)
    {
        case STABILIZE:
            while (xSemaphoreTake(this->stab_PIDs_semaphore, 0) != pdTRUE);

            for (uint8_t i = ROLL; i <= YAW; i++)
            {
                this->stab_PIDs[i]->Set_Kp(parameters[i][0]);
                this->stab_PIDs[i]->Set_Ki(parameters[i][1]);
                this->stab_PIDs[i]->Set_Kd(parameters[i][2]);
            }

            xSemaphoreGive(this->stab_PIDs_semaphore);
            break;
        case RATE:
            while (xSemaphoreTake(this->rate_PIDs_semaphore, 0) != pdTRUE);

            for (uint8_t i = ROLL; i <= YAW; i++)
            {
                this->rate_PIDs[i]->Set_Kp(parameters[i][0]);
                this->rate_PIDs[i]->Set_Ki(parameters[i][1]);
                this->rate_PIDs[i]->Set_Kd(parameters[i][2]);
            }

            xSemaphoreGive(this->rate_PIDs_semaphore);
            break;
    }
}

void Motors_Controller::Set_Throttle(uint16_t throttle)
{
    if (throttle > 1000)
        return;

    while (xSemaphoreTake(this->throttle_semaphore, 0) != pdTRUE);

    this->throttle = throttle;

    xSemaphoreGive(this->throttle_semaphore);
}

void Motors_Controller::Set_Invert_Yaw(bool invert)
{
    // TODO semaphore might not be needed since boolean should be atomic
    while (xSemaphoreTake(this->invert_yaw_semaphore, 0) != pdTRUE);

    this->invert_yaw = invert;

    xSemaphoreGive(this->invert_yaw_semaphore);
}

void Motors_Controller::Update_Motors(IMU::Euler_Angles euler, IMU::Sensor_Data gyro)
{
    // consider more than 60 deg unsafe
    if (std::fabs(euler.roll) > 60 || std::fabs(euler.pitch) > 60)
        esp_restart();

    while (xSemaphoreTake(this->throttle_semaphore, 0) != pdTRUE);

    uint16_t throttle = this->throttle;

    xSemaphoreGive(this->throttle_semaphore);


    if (throttle > 180)
    {
        double stab_corrections[3], rate_corrections[3];

        while (xSemaphoreTake(this->stab_PIDs_semaphore, 0) != pdTRUE);

        stab_corrections[ROLL] = this->stab_PIDs[ROLL]->Get_PID(0 - euler.roll);
        stab_corrections[PITCH] = this->stab_PIDs[PITCH]->Get_PID(0 - euler.pitch);
        stab_corrections[YAW] = 0;
        //stab_corrections[YAW] = this->stab_PIDs[YAW]->Get_PID(this->reference_yaw - euler.yaw);

        xSemaphoreGive(this->stab_PIDs_semaphore);

        while (xSemaphoreTake(this->rate_PIDs_semaphore, 0) != pdTRUE);

        rate_corrections[ROLL] = this->roll_filter->Apply_Filter(
                this->rate_PIDs[ROLL]->Get_PID(stab_corrections[ROLL] - gyro.y));
        rate_corrections[PITCH] = this->pitch_filter->Apply_Filter(
                this->rate_PIDs[PITCH]->Get_PID(stab_corrections[PITCH] - gyro.x));
        rate_corrections[YAW] = this->yaw_filter->Apply_Filter(
                this->rate_PIDs[YAW]->Get_PID(stab_corrections[YAW] - gyro.z));

        xSemaphoreGive(this->rate_PIDs_semaphore);

        while (xSemaphoreTake(this->invert_yaw_semaphore, 0) != pdTRUE);

        // not sure about yaw signs
        // FL + BR -> counterclockwise
        // FR + BL -> clockwise
        if (!this->invert_yaw)
        {
            xSemaphoreGive(this->invert_yaw_semaphore);

            this->motor_FL = throttle - rate_corrections[ROLL] - rate_corrections[PITCH]
                             - rate_corrections[YAW];
            this->motor_BL = throttle - rate_corrections[ROLL] + rate_corrections[PITCH]
                             + rate_corrections[YAW];
            this->motor_FR = throttle + rate_corrections[ROLL] - rate_corrections[PITCH]
                             + rate_corrections[YAW];
            this->motor_BR = throttle + rate_corrections[ROLL] + rate_corrections[PITCH]
                             - rate_corrections[YAW];
        } else
        {
            xSemaphoreGive(this->invert_yaw_semaphore);

            this->motor_FL = throttle - rate_corrections[ROLL] - rate_corrections[PITCH]
                             + rate_corrections[YAW];
            this->motor_BL = throttle - rate_corrections[ROLL] + rate_corrections[PITCH]
                             - rate_corrections[YAW];
            this->motor_FR = throttle + rate_corrections[ROLL] - rate_corrections[PITCH]
                             - rate_corrections[YAW];
            this->motor_BR = throttle + rate_corrections[ROLL] + rate_corrections[PITCH]
                             + rate_corrections[YAW];
        }

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

        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FL, this->motor_FL);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BL, this->motor_BL);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FR, this->motor_FR);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BR, this->motor_BR);
    } else
    {
        this->reference_yaw = euler.yaw;

        // TODO
        //MPU6050::Instance().Reset_Integrators();

        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FL, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BL, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FR, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BR, 0);
    }
}

} /* namespace flyhero */
