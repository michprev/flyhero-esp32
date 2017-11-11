/*
 * Math.h
 *
 *  Created on: 9. 9. 2017
 *      Author: michp
 */

#pragma once

#include <cmath>


namespace flyhero
{

class Math
{
private:
    Math() {};

public:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr float DEG_TO_RAD = Math::PI / 180;
    static constexpr float RAD_TO_DEG = 180 / Math::PI;

    static inline float Inv_Sqrt(float x);
    static float Atan2(float y, float x);
};

// https://en.wikipedia.org/wiki/Fast_inverse_square_root
inline float Math::Inv_Sqrt(float x)
{
    float y = x;
    long i = *(long *) &y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (0.5f * x * y * y));

    return y;
}

// use Betaflight atan2 approx: https://github.com/betaflight/betaflight/blob/master/src/main/common/maths.c
inline float Math::Atan2(float y, float x)
{
    const float atanPolyCoef1 = 3.14551665884836e-07f;
    const float atanPolyCoef2 = 0.99997356613987f;
    const float atanPolyCoef3 = 0.14744007058297684f;
    const float atanPolyCoef4 = 0.3099814292351353f;
    const float atanPolyCoef5 = 0.05030176425872175f;
    const float atanPolyCoef6 = 0.1471039133652469f;
    const float atanPolyCoef7 = 0.6444640676891548f;

    float abs_x, abs_y;
    float result;

    abs_x = std::abs(x);
    abs_y = std::abs(y);

    result = (abs_x > abs_y ? abs_x : abs_y);

    if (result != 0)
        result = (abs_x < abs_y ? abs_x : abs_y) / result;

    result = -((((atanPolyCoef5 * result - atanPolyCoef4) * result - atanPolyCoef3) * result - atanPolyCoef2) * result -
               atanPolyCoef1) / ((atanPolyCoef7 * result + atanPolyCoef6) * result + 1.0);
    result *= Math::RAD_TO_DEG;

    if (abs_y > abs_x)
        result = 90 - result;
    if (x < 0)
        result = 180 - result;
    if (y < 0)
        result = -result;

    return result;
}

} /* namespace flyhero */
