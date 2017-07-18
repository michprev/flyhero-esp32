/*
 * Motors_Controller.h
 *
 *  Created on: 18. 7. 2017
 *      Author: michp
 */

#ifndef MOTORS_CONTROLLER_H_
#define MOTORS_CONTROLLER_H_

#include "PID.h"
#include "PWM_Generator.h"
#include "MPU6050.h"

namespace flyhero {

enum Axis { Roll, Pitch, Yaw };

class Motors_Controller {
private:
	Motors_Controller();
	Motors_Controller(Motors_Controller const&){};
	Motors_Controller& operator=(Motors_Controller const&){};

	PID roll_PID, pitch_PID, yaw_PID;
	uint16_t motor_FL, motor_FR, motor_BL, motor_BR;
	uint16_t throttle;
	bool invert_yaw;

public:
	static Motors_Controller& Instance();

	void Set_PID_Constants(Axis axis, float Kp, float Ki, float Kd);
	void Set_Throttle(uint16_t throttle);
	void Set_Invert_Yaw(bool invert);
	void Update_Motors();

	uint16_t Get_Throttle();
};

} /* namespace flyhero */

#endif /* MOTORS_CONTROLLER_H_ */
