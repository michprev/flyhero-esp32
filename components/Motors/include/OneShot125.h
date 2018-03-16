//
// Created by michal on 7.3.18.
//

#pragma once

#include "Motors_Protocol.h"

namespace flyhero
{

class OneShot125 : Motors_Protocol
{
private:
    OneShot125() = default;

    OneShot125(OneShot125 const &);

    OneShot125 &operator=(OneShot125 const &);

public:
    static Motors_Protocol &Instance();

    void Init() override;

    void Arm() override;

    void Update(uint16_t motor_fl, uint16_t motor_bl, uint16_t motor_fr, uint16_t motor_br) override;
};

}