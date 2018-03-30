//
// Created by michal on 7.3.18.
//

#pragma once

#include <cstdint>

namespace flyhero
{

class Motors_Protocol
{
public:
    virtual void Init() = 0;

    virtual void Arm() = 0;

    virtual void Disarm() = 0;

    virtual void Update(uint16_t motor_fl, uint16_t motor_bl, uint16_t motor_fr, uint16_t motor_br) = 0;
};

}