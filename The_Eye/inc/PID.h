/*
 * PID.h
 *
 *  Created on: 14. 7. 2017
 *      Author: michp
 */

#ifndef PID_H_
#define PID_H_

#include "cmath"
#include "Biquad_Filter.h"
#include "Timer.h"

namespace flyhero {

class PID {
private:
	Biquad_Filter d_term_lpf;
	uint32_t last_t;
	float Kp, Ki, Kd;
	float integrator;
	float i_max;
	float last_d;
	float last_error;

public:
	PID(float i_max = 0, float Kp = 0, float Ki = 0, float Kd = 0);

	float Get_PID(float error);
	void Set_Kp(float Kp);
	void Set_Ki(float Ki);
	void Set_Kd(float Kd);
	void Set_I_Max(float i_max);
};

} /* namespace flyhero */

#endif /* PID_H_ */
