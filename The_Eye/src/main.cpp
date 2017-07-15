/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Michal Prevratil
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "PWM_Generator.h"
#include "ESP.h"
#include "MS5611.h"
#include "MPU6050.h"
#include "LEDs.h"
#include "NEO_M8N.h"
#include "PID.h"
#include "Logger.h"
#include "Timer.h"
#include "ESP_Connection.h"

using namespace flyhero;

#ifdef LOG
extern "C" void initialise_monitor_handles(void);
#endif

ESP& esp = ESP::Create_Instance(ESP8266);
PWM_Generator& pwm = PWM_Generator::Instance();
MPU6050& mpu = MPU6050::Instance();
MS5611& ms5611 = MS5611::Instance();
NEO_M8N& neo = NEO_M8N::Instance();
Logger& logger = Logger::Instance();

void Arm_Callback();
void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length);
void IMU_Data_Ready_Callback();
void IMU_Data_Read_Callback();

PID PID_Roll(50);
PID PID_Pitch(50);
PID PID_Yaw(50);
bool connected = false;
bool start = false;
bool inverse_yaw = false;
uint16_t throttle = 0;
IWDG_HandleTypeDef hiwdg;

volatile bool data_received = false;
volatile bool log_flag = false;

int main(void)
{
	HAL_Init();
#ifdef LOG
	initialise_monitor_handles();
#endif

	uint32_t timestamp;

	LEDs::Init();

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 480;
	// timeout after 2 s

	// reset gyro
	if (mpu.Init()) {
		LEDs::TurnOn(LEDs::Yellow);
		while (true);
	}

	logger.Init();
	esp.Init(&IPD_Callback);

	timestamp = HAL_GetTick();

	while (!connected) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp.Process_Data();
	}
	LEDs::TurnOff(LEDs::Green);

	pwm.Init();
	pwm.Arm(&Arm_Callback);

	if (mpu.Calibrate() != HAL_OK) {
		LEDs::TurnOn(LEDs::Yellow);
		while (true);
	}

	pwm.SetPulse(1100, 1);
	pwm.SetPulse(1100, 2);
	pwm.SetPulse(1100, 3);
	pwm.SetPulse(1100, 4);

	Timer::Delay_ms(250);

	pwm.SetPulse(940, 4);
	pwm.SetPulse(940, 1);
	pwm.SetPulse(940, 3);
	pwm.SetPulse(940, 2);

	while (!start) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp.Process_Data();
	}

	LEDs::TurnOn(LEDs::Green);

#ifndef LOG
	HAL_IWDG_Init(&hiwdg);
#endif

	throttle = 0;

	mpu.Data_Ready_Callback = &IMU_Data_Ready_Callback;
	mpu.Data_Read_Callback = &IMU_Data_Read_Callback;

	while (true) {
		if (log_flag) {
			log_flag = false;

			// 200 us
			logger.Send_Data();
		}
		esp.Get_Connection('4')->Connection_Send_Continue();
	}
}

void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length) {
	switch (length) {
	case 22:
		// around 75 us
		if (data[0] == 0x5D) {
			uint16_t roll_kP, pitch_kP, yaw_kP;
			uint16_t roll_kI, pitch_kI, yaw_kI;
			uint16_t roll_kD, pitch_kD, yaw_kD;

			data_received = true;

			// 3 us
			throttle = data[1] << 8;
			throttle |= data[2];

			// 3 us
			roll_kP = data[3] << 8;
			roll_kP |= data[4];

			roll_kI = data[5] << 8;
			roll_kI |= data[6];

			roll_kD = data[7] << 8;
			roll_kD |= data[8];

			// 29 us
			PID_Roll.Set_Kp(roll_kP * 0.01f);
			PID_Roll.Set_Ki(roll_kI * 0.01f);
			PID_Roll.Set_Kd(roll_kD * 0.01f);

			pitch_kP = data[9] << 8;
			pitch_kP |= data[10];

			pitch_kI = data[11] << 8;
			pitch_kI |= data[12];

			pitch_kD = data[13] << 8;
			pitch_kD |= data[14];

			PID_Pitch.Set_Kp(pitch_kP * 0.01f);
			PID_Pitch.Set_Ki(pitch_kI * 0.01f);
			PID_Pitch.Set_Kd(pitch_kD * 0.01f);

			yaw_kP = data[15] << 8;
			yaw_kP |= data[16];

			yaw_kI = data[17] << 8;
			yaw_kI |= data[18];

			yaw_kD = data[19] << 8;
			yaw_kD |= data[20];

			PID_Yaw.Set_Kp(yaw_kP * 0.01f);
			PID_Yaw.Set_Ki(yaw_kI * 0.01f);
			PID_Yaw.Set_Kd(yaw_kD * 0.01f);

			inverse_yaw = (data[21] == 0x01);

			throttle += 1000;
		}
		break;
	case 3:
		if (data[0] == 0x3D) {
			start = true;
		}
		if (data[0] == 0x5D) {
			connected = true;
			uint16_t log_options = (data[1] << 8) | data[2];

			logger.Set_Data_Type(Logger::UART, (Logger::Data_Type)log_options);
		}
		break;
	}
}

void Arm_Callback() {
	esp.Process_Data();
	//HAL_IWDG_Refresh(&hiwdg);
}

void IMU_Data_Ready_Callback() {
	// 160 us
	if (mpu.Start_Read() != HAL_OK)
		LEDs::TurnOn(LEDs::Orange);
}

long FL, BL, FR, BR;
long pitch_correction, roll_correction, yaw_correction;
MPU6050::Sensor_Data euler_data;

// IMU_Data_Ready_Callback() -> 340 us -> IMU_Data_Read_Callback()

void IMU_Data_Read_Callback() {
	if (data_received)
		HAL_IWDG_Refresh(&hiwdg);
	data_received = false;

	mpu.Complete_Read();
	// 200 us
	mpu.Compute_Euler();
	mpu.Get_Euler(euler_data.x, euler_data.y, euler_data.z);

	log_flag = true;

	// 73 us
	if (throttle >= 1050) {
		// 50 us
		roll_correction = PID_Roll.Get_PID(euler_data.x);
		pitch_correction = PID_Pitch.Get_PID(euler_data.y);
		yaw_correction = PID_Yaw.Get_PID(euler_data.z);

		// not sure about yaw signs
		if (!inverse_yaw) {
			FL = throttle + roll_correction + yaw_correction; // PB2
			BL = throttle - pitch_correction - yaw_correction; // PA15
			FR = throttle + pitch_correction - yaw_correction; // PB10
			BR = throttle - roll_correction + yaw_correction; // PA1
		}
		else {
			FL = throttle + roll_correction - yaw_correction; // PB2
			BL = throttle - pitch_correction + yaw_correction; // PA15
			FR = throttle + pitch_correction + yaw_correction; // PB10
			BR = throttle - roll_correction - yaw_correction; // PA1
		}

		if (FL > 2000)
			FL = 2000;
		else if (FL < 1050)
			FL = 940;

		if (BL > 2000)
			BL = 2000;
		else if (BL < 1050)
			BL = 940;

		if (FR > 2000)
			FR = 2000;
		else if (FR < 1050)
			FR = 940;

		if (BR > 2000)
			BR = 2000;
		else if (BR < 1050)
			BR = 940;

		// 17 us
		pwm.SetPulse(FL, 3);
		pwm.SetPulse(BL, 2);
		pwm.SetPulse(FR, 4);
		pwm.SetPulse(BR, 1);
	}
	else {
		mpu.Reset_Integrators();

		pwm.SetPulse(940, 4);
		pwm.SetPulse(940, 1);
		pwm.SetPulse(940, 3);
		pwm.SetPulse(940, 2);
	}
}
