/*
 * PID.h
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#pragma once

#include <esp_timer.h>
#include <cmath>

#include "Biquad_Filter.h"


namespace flyhero
{

class PID
{
private:
    Biquad_Filter d_term_lpf;
    int64_t last_t;
    double Kp, Ki, Kd;
    double integrator;
    double i_max;
    double last_d;
    double last_error;

public:
    PID(double update_rate, double i_max = 0, double Kp = 0, double Ki = 0, double Kd = 0);

    double Get_PID(double error);

    void Set_Kp(double Kp);

    void Set_Ki(double Ki);

    void Set_Kd(double Kd);

    void Set_I_Max(double i_max);
};

} /* namespace flyhero */
