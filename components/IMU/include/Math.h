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
    Math();

public:
    static constexpr double DEG_TO_RAD = M_PI / 180;
    static constexpr double RAD_TO_DEG = 180 * M_1_PI;
};

} /* namespace flyhero */
