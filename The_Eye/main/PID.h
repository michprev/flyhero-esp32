/*
 * PID.h
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#pragma once

#include <sys/time.h>
#include <cmath>

#include "Biquad_Filter.h"


namespace flyhero
{

class PID
{
private:
    Biquad_Filter d_term_lpf;
    timeval last_t;
    float Kp, Ki, Kd;
    float integrator;
    float i_max;
    float last_d;
    float last_error;

public:
    PID(float update_rate, float i_max = 0, float Kp = 0, float Ki = 0, float Kd = 0);

    float Get_PID(float error);
    void Set_Kp(float Kp);
    void Set_Ki(float Ki);
    void Set_Kd(float Kd);
    void Set_I_Max(float i_max);
};

} /* namespace flyhero */
