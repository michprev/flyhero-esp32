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
    static constexpr float PI = 3.14159265358979323846f;
    static constexpr float DEG_TO_RAD = PI / 180;
    static constexpr float RAD_TO_DEG = 180 / PI;
};

} /* namespace flyhero */
