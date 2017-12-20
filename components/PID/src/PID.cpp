/*
 * PID.cpp
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#include <cstdlib>
#include "PID.h"


namespace flyhero
{

PID::PID(double update_rate, double i_max, double Kp, double Ki, double Kd)
        : d_term_lpf(Biquad_Filter::FILTER_LOW_PASS, update_rate, 20)
{
    this->last_t = 0;
    this->integrator = 0;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->i_max = i_max;
    this->last_d = NAN;
    this->last_error = NAN;
}

double PID::Get_PID(double error)
{
    int64_t now = esp_timer_get_time();

    double delta_t = (now - this->last_t) * 0.000001;

    if (this->last_t == 0 || delta_t > 1)
    {
        this->integrator = 0;
        delta_t = 0;
    }

    this->last_t = now;

    double output = 0;

    // proportional component
    output += error * this->Kp;

    // integral component
    if (this->Ki != 0 && delta_t > 0)
    {
        this->integrator += error * this->Ki * delta_t;

        if (this->integrator < -this->i_max)
            this->integrator = -this->i_max;
        if (this->integrator > this->i_max)
            this->integrator = this->i_max;

        output += this->integrator;
    }

    // derivative component
    if (this->Kd != 0 && delta_t > 0)
    {
        double derivative;

        if (std::isnan(this->last_d))
        {
            derivative = 0;
            this->last_d = 0;
        } else
            derivative = (error - this->last_error) / delta_t;

        // apply 20 Hz biquad LPF
        derivative = this->d_term_lpf.Apply_Filter(derivative);

        this->last_error = error;
        this->last_d = derivative;

        output += derivative * this->Kd;
    }

    return output;
}

void PID::Set_Kp(double Kp)
{
    this->Kp = Kp;
}

void PID::Set_Ki(double Ki)
{
    this->Ki = Ki;
}

void PID::Set_Kd(double Kd)
{
    this->Kd = Kd;
}

void PID::Set_I_Max(double i_max)
{
    this->i_max = i_max;
}

} /* namespace flyhero */
