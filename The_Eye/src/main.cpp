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
Motors_Controller& motors_controller = Motors_Controller::Instance();

void Arm_Callback();
void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length);
void IMU_Data_Ready_Callback();
void IMU_Data_Read_Callback();

bool connected = false;
bool start = false;
bool inverse_yaw = false;
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
			uint16_t roll_Kp, pitch_Kp, yaw_Kp;
			uint16_t roll_Ki, pitch_Ki, yaw_Ki;
			uint16_t roll_Kd, pitch_Kd, yaw_Kd;
			uint16_t throttle;

			data_received = true;

			// 3 us
			throttle = data[1] << 8;
			throttle |= data[2];

			motors_controller.Set_Throttle(throttle + 1000);

			// 3 us
			roll_Kp = data[3] << 8;
			roll_Kp |= data[4];

			roll_Ki = data[5] << 8;
			roll_Ki |= data[6];

			roll_Kd = data[7] << 8;
			roll_Kd |= data[8];

			motors_controller.Set_PID_Constants(Roll, roll_Kp * 0.01f, roll_Ki * 0.01f, roll_Kd * 0.01f);

			pitch_Kp = data[9] << 8;
			pitch_Kp |= data[10];

			pitch_Ki = data[11] << 8;
			pitch_Ki |= data[12];

			pitch_Kd = data[13] << 8;
			pitch_Kd |= data[14];

			motors_controller.Set_PID_Constants(Pitch, pitch_Kp * 0.01f, pitch_Ki * 0.01f, pitch_Kd * 0.01f);

			yaw_Kp = data[15] << 8;
			yaw_Kp |= data[16];

			yaw_Ki = data[17] << 8;
			yaw_Ki |= data[18];

			yaw_Kd = data[19] << 8;
			yaw_Kd |= data[20];

			motors_controller.Set_PID_Constants(Yaw, yaw_Kp * 0.01f, yaw_Ki * 0.01f, yaw_Kd * 0.01f);
			motors_controller.Set_Invert_Yaw(data[21] == 0x01);
		}
		break;
	case 3:
		if (data[0] == 0x3D) {
			start = true;
		}
		if (data[0] == 0x5D) {
			connected = true;
			uint16_t log_options = (data[1] << 8) | data[2];

			logger.Set_Data_Type(Logger::WiFi, (Logger::Data_Type)log_options);
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

// IMU_Data_Ready_Callback() -> 340 us -> IMU_Data_Read_Callback()

void IMU_Data_Read_Callback() {
	if (data_received)
		HAL_IWDG_Refresh(&hiwdg);
	data_received = false;
	log_flag = true;

	mpu.Complete_Read();
	mpu.Compute_Mahony();
	//mpu.Compute_Euler();

	motors_controller.Update_Motors();
}
