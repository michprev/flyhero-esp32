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

    this->invert_yaw = false;
    this->throttle = 0;

    this->roll_PID_semaphore = xSemaphoreCreateBinary();
    this->pitch_PID_semaphore = xSemaphoreCreateBinary();
    this->yaw_PID_semaphore = xSemaphoreCreateBinary();
    this->throttle_semaphore = xSemaphoreCreateBinary();
    this->invert_yaw_semaphore = xSemaphoreCreateBinary();
}

void Motors_Controller::Init()
{
    double sample_rate = IMU_Detector::Detect_IMU().Get_Sample_Rate();

    this->roll_PID = new PID(sample_rate);
    this->pitch_PID = new PID(sample_rate);
    this->yaw_PID = new PID(sample_rate);

    this->roll_PID->Set_I_Max(50);
    this->pitch_PID->Set_I_Max(50);
    this->yaw_PID->Set_I_Max(50);

    xSemaphoreGive(this->roll_PID_semaphore);
    xSemaphoreGive(this->pitch_PID_semaphore);
    xSemaphoreGive(this->yaw_PID_semaphore);
    xSemaphoreGive(this->throttle_semaphore);
    xSemaphoreGive(this->invert_yaw_semaphore);

    this->pwm.Init();
    this->pwm.Arm();
}

void Motors_Controller::Set_PID_Constants(Axis axis, double Kp, double Ki, double Kd)
{
    switch (axis)
    {
        case Roll:
            while (xSemaphoreTake(this->roll_PID_semaphore, 0) != pdTRUE);

            this->roll_PID->Set_Kp(Kp);
            this->roll_PID->Set_Ki(Ki);
            this->roll_PID->Set_Kd(Kd);

            xSemaphoreGive(this->roll_PID_semaphore);
            break;
        case Pitch:
            while (xSemaphoreTake(this->pitch_PID_semaphore, 0) != pdTRUE);

            this->pitch_PID->Set_Kp(Kp);
            this->pitch_PID->Set_Ki(Ki);
            this->pitch_PID->Set_Kd(Kd);

            xSemaphoreGive(this->pitch_PID_semaphore);
            break;
        case Yaw:
            while (xSemaphoreTake(this->yaw_PID_semaphore, 0) != pdTRUE);

            this->yaw_PID->Set_Kp(Kp);
            this->yaw_PID->Set_Ki(Ki);
            this->yaw_PID->Set_Kd(Kd);

            xSemaphoreGive(this->yaw_PID_semaphore);
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

void Motors_Controller::Update_Motors(IMU::Euler_Angles euler)
{
    while (xSemaphoreTake(this->throttle_semaphore, 0) != pdTRUE);

    uint16_t throttle = this->throttle;

    xSemaphoreGive(this->throttle_semaphore);


    if (throttle > 0)
    {
        double pitch_correction, roll_correction, yaw_correction;

        while (xSemaphoreTake(this->roll_PID_semaphore, 0) != pdTRUE);
        roll_correction = this->roll_PID->Get_PID(0 - euler.roll);
        xSemaphoreGive(this->roll_PID_semaphore);

        while (xSemaphoreTake(this->pitch_PID_semaphore, 0) != pdTRUE);
        pitch_correction = this->pitch_PID->Get_PID(0 - euler.pitch);
        xSemaphoreGive(this->pitch_PID_semaphore);

        while (xSemaphoreTake(this->yaw_PID_semaphore, 0) != pdTRUE);
        yaw_correction = this->yaw_PID->Get_PID(0 - euler.yaw);
        xSemaphoreGive(this->yaw_PID_semaphore);

        while (xSemaphoreTake(this->invert_yaw_semaphore, 0) != pdTRUE);

        // not sure about yaw signs
        if (!this->invert_yaw)
        {
            xSemaphoreGive(this->invert_yaw_semaphore);

            this->motor_FL = throttle - roll_correction - pitch_correction - yaw_correction; // PB2
            this->motor_BL = throttle - roll_correction + pitch_correction + yaw_correction; // PA15
            this->motor_FR = throttle + roll_correction - pitch_correction + yaw_correction; // PB10
            this->motor_BR = throttle + roll_correction + pitch_correction - yaw_correction; // PA1
        } else
        {
            xSemaphoreGive(this->invert_yaw_semaphore);

            this->motor_FL = throttle - roll_correction - pitch_correction + yaw_correction; // PB2
            this->motor_BL = throttle - roll_correction + pitch_correction - yaw_correction; // PA15
            this->motor_FR = throttle + roll_correction - pitch_correction - yaw_correction; // PB10
            this->motor_BR = throttle + roll_correction + pitch_correction + yaw_correction; // PA1
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
        // TODO
        //MPU6050::Instance().Reset_Integrators();

        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FL, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BL, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_FR, 0);
        this->pwm.Set_Pulse(PWM_Generator::MOTOR_BR, 0);
    }
}

} /* namespace flyhero */
