/*
 * Motors_Controller.cpp
 *
 *  Created on: 18. 7. 2017
 *      Author: michp
 */

#include <Motors_Controller.h>

namespace flyhero {

Motors_Controller& Motors_Controller::Instance() {
	static Motors_Controller instance;

	return instance;
}

Motors_Controller::Motors_Controller() {
	this->motor_FR = 940;
	this->motor_FL = 940;
	this->motor_BR = 940;
	this->motor_BL = 940;

	this->roll_PID.Set_I_Max(50);
	this->pitch_PID.Set_I_Max(50);
	this->yaw_PID.Set_I_Max(50);

	this->invert_yaw = false;
	this->throttle = 1000;
}

void Motors_Controller::Set_PID_Constants(Axis axis, float Kp, float Ki, float Kd) {
	switch (axis) {
	case Roll:
		this->roll_PID.Set_Kp(Kp);
		this->roll_PID.Set_Ki(Ki);
		this->roll_PID.Set_Kd(Kd);
		break;
	case Pitch:
		this->pitch_PID.Set_Kp(Kp);
		this->pitch_PID.Set_Ki(Ki);
		this->pitch_PID.Set_Kd(Kd);
		break;
	case Yaw:
		this->yaw_PID.Set_Kp(Kp);
		this->yaw_PID.Set_Ki(Ki);
		this->yaw_PID.Set_Kd(Kd);
		break;
	}
}

void Motors_Controller::Set_Throttle(uint16_t throttle) {
	this->throttle = throttle;
}

void Motors_Controller::Set_Invert_Yaw(bool invert) {
	this->invert_yaw = invert;
}

void Motors_Controller::Update_Motors() {
	PWM_Generator& PWM_generator = PWM_Generator::Instance();

	if (this->throttle >= 1050) {
		float pitch_correction, roll_correction, yaw_correction;
		MPU6050::Sensor_Data euler_data;

		MPU6050::Instance().Get_Euler(euler_data.x, euler_data.y, euler_data.z);

		roll_correction = this->roll_PID.Get_PID(0 - euler_data.x);
		pitch_correction = this->pitch_PID.Get_PID(0 - euler_data.y);
		yaw_correction = this->yaw_PID.Get_PID(0 - euler_data.z);

		// not sure about yaw signs
		if (!this->invert_yaw) {
			this->motor_FL = throttle - roll_correction - pitch_correction - yaw_correction; // PB2
			this->motor_BL = throttle - roll_correction + pitch_correction + yaw_correction; // PA15
			this->motor_FR = throttle + roll_correction - pitch_correction + yaw_correction; // PB10
			this->motor_BR = throttle + roll_correction + pitch_correction - yaw_correction; // PA1
		}
		else {
			this->motor_FL = throttle - roll_correction - pitch_correction + yaw_correction; // PB2
			this->motor_BL = throttle - roll_correction + pitch_correction - yaw_correction; // PA15
			this->motor_FR = throttle + roll_correction - pitch_correction - yaw_correction; // PB10
			this->motor_BR = throttle + roll_correction + pitch_correction + yaw_correction; // PA1
		}

		if (this->motor_FL > 2000)
			this->motor_FL = 2000;
		else if (this->motor_FL < 1050)
			this->motor_FL = 940;

		if (this->motor_BL > 2000)
			this->motor_BL = 2000;
		else if (this->motor_BL < 1050)
			this->motor_BL = 940;

		if (this->motor_FR > 2000)
			this->motor_FR = 2000;
		else if (this->motor_FR < 1050)
			this->motor_FR = 940;

		if (this->motor_BR > 2000)
			this->motor_BR = 2000;
		else if (this->motor_BR < 1050)
			this->motor_BR = 940;

		PWM_generator.SetPulse(this->motor_FL, 3);
		PWM_generator.SetPulse(this->motor_BL, 2);
		PWM_generator.SetPulse(this->motor_FR, 4);
		PWM_generator.SetPulse(this->motor_BR, 1);
	}
	else {
		MPU6050::Instance().Reset_Integrators();

		PWM_generator.SetPulse(940, 1);
		PWM_generator.SetPulse(940, 2);
		PWM_generator.SetPulse(940, 3);
		PWM_generator.SetPulse(940, 4);
	}
}

uint16_t Motors_Controller::Get_Throttle() {
	return this->throttle;
}

uint16_t Motors_Controller::Get_Motor_FL() {
	return this->motor_FL;
}

uint16_t Motors_Controller::Get_Motor_FR() {
	return this->motor_FR;
}

uint16_t Motors_Controller::Get_Motor_BL() {
	return this->motor_BL;
}

uint16_t Motors_Controller::Get_Motor_BR() {
	return this->motor_BR;
}

} /* namespace flyhero */
