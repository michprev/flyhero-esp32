/*
 * PID.cpp
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#include <PID.h>

namespace flyhero {

// assume that PID will be computed at 1 kHz
PID::PID(float i_max, float Kp, float Ki, float Kd)
	: d_term_lpf(Biquad_Filter::FILTER_LOW_PASS, 1000, 20)
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

float PID::Get_PID(float error) {
	// ticks in us
	float dt = (Timer::Get_Tick_Count() - this->last_t) * 0.000001;
	float output = 0;

	if (this->last_t == 0 || dt > 1000) {
		this->integrator = 0;
		dt = 0;
	}

	this->last_t = Timer::Get_Tick_Count();

	// proportional component
	output += error * this->Kp;

	// integral component
	if (this->Ki != 0 && dt > 0) {
		this->integrator += error * this->Ki * dt;

		if (this->integrator < -this->i_max)
			this->integrator = -this->i_max;
		if (this->integrator > this->i_max)
			this->integrator = this->i_max;

		output += this->integrator;
	}

	// derivative component
	if (this->Kd != 0 && dt > 0) {
		float derivative;

		if (std::isnan(this->last_d)) {
			derivative = 0;
			this->last_d = 0;
		}
		else
			derivative = (error - this->last_error) / dt;

		// apply 20 Hz biquad LPF
		derivative = this->d_term_lpf.Apply_Filter(derivative);

		this->last_error = error;
		this->last_d = derivative;

		output += derivative * this->Kd;
	}

	return output;
}

void PID::Set_Kp(float Kp) {
	this->Kp = Kp;
}

void PID::Set_Ki(float Ki) {
	this->Ki = Ki;
}

void PID::Set_Kd(float Kd) {
	this->Kd = Kd;
}

void PID::Set_I_Max(float i_max) {
	this->i_max = i_max;
}

} /* namespace flyhero */
