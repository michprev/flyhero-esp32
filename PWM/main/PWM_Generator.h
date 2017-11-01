/*
 * PWM_Generator.h
 *
 *  Created on: 11. 12. 2016
 *      Author: michp
 */

#pragma once

#include <driver/mcpwm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


namespace flyhero
{

class PWM_Generator
{
private:
    PWM_Generator() {};
    PWM_Generator(PWM_Generator const &);
    PWM_Generator &operator=(PWM_Generator const &);

public:
    enum Motor_Type
    {
        MOTOR_FL = 1 << 0, MOTOR_BL = 1 << 1, MOTOR_FR = 1 << 2,
        MOTOR_BR = 1 << 3, MOTORS_ALL = MOTOR_FL | MOTOR_BL | MOTOR_FR | MOTOR_BR
    };

    static PWM_Generator &Instance();
    void Init();
    void Set_Pulse(Motor_Type motor, uint16_t us);
    void Arm();
};

inline PWM_Generator::Motor_Type operator|(PWM_Generator::Motor_Type a, PWM_Generator::Motor_Type b)
{
    return static_cast<PWM_Generator::Motor_Type>(static_cast<int>(a) | static_cast<int>(b));
}

}
