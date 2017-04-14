/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Michal Prevratil
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "main.h"
#include "ESP8266.h"
#include "Kalman.h"
#include "math.h"

using namespace The_Eye;


#define PI 3.14159265358979323846
//#define LOG

#ifdef LOG
extern "C" void initialise_monitor_handles(void);
#endif

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

ESP *esp = ESP8266::Instance();
PWM_Generator *pwm = PWM_Generator::Instance();
MPU9250 *mpu = MPU9250::Instance();
MS5611 *ms5611 = MS5611::Instance();
NEO_M8N *neo = NEO_M8N::Instance();
Logger *logger = Logger::Instance();

void InitI2C(I2C_HandleTypeDef*);
void Arm_Callback();
void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length);
void Calculate();

PID PID_Roll, PID_Pitch, PID_Yaw;
uint16_t rollKp, pitchKp, yawKp;
bool connected = false;
bool start = false;
bool data_received = false;
uint16_t throttle = 0;
IWDG_HandleTypeDef hiwdg;

/* Testing */
Kalman kalmanX, kalmanY;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t kalman_time;
double dt;
MPU9250::Sensor_Data accel_data;
MPU9250::Sensor_Data gyro_data;
MPU9250::Sensor_Data euler_data;
double roll, pitch;

/* End Testing */

int main(void)
{
	HAL_Init();
#ifdef LOG
	initialise_monitor_handles();
#endif
	uint8_t status;
	uint32_t timestamp;
	//int32_t press, temp;
	float data[3];
	uint8_t accuracy;
	long FL, BL, FR, BR;
	FL = BL = FR = BR = 0;
	long pitchCorrection, rollCorrection, yawCorrection;

	LEDs::Init();
#ifdef LOG
	logger->Init();
#endif

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 480;
	// timeout after 2 s

	I2C_HandleTypeDef hI2C_Handle;
	InitI2C(&hI2C_Handle);

	esp->IPD_Callback = &IPD_Callback;
	esp->Init();

	timestamp = HAL_GetTick();
	while (!connected) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp->Process_Data();
	}
	LEDs::TurnOff(LEDs::Green);

	pwm->Init();
	pwm->Arm(&Arm_Callback);

	//neo->Init();

	// reset barometer
	/*if (ms5611->Init(&hI2C_Handle) != HAL_OK) {
		LEDs::TurnOn(LEDs::Orange);
		while (true);
	}*/

	// reset gyro
	if (mpu->Init(&hI2C_Handle)) {
		LEDs::TurnOn(LEDs::Yellow);
		while (true);
	}

	if (mpu->SelfTest())
		LEDs::TurnOn(LEDs::Green);
	else
		LEDs::TurnOn(LEDs::Yellow);

	pwm->SetPulse(1100, 1);
	pwm->SetPulse(1100, 2);
	pwm->SetPulse(1100, 3);
	pwm->SetPulse(1100, 4);

	HAL_Delay(250);

	pwm->SetPulse(940, 4);
	pwm->SetPulse(940, 1);
	pwm->SetPulse(940, 3);
	pwm->SetPulse(940, 2);

	while (!start) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		mpu->CheckNewData();
		esp->Process_Data();
	}

	while (!mpu->ReadAccel(&accel_data));
	roll  = atan2(accel_data.y, accel_data.z) * 180 / PI;
	pitch = atan(-accel_data.x / sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180 / PI;

	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compAngleX = roll;
	compAngleY = pitch;

	kalman_time = HAL_GetTick();


	LEDs::TurnOn(LEDs::Green);

	//ms5611->ConvertD1();
#ifndef LOG
	HAL_IWDG_Init(&hiwdg);
#endif

	timestamp = HAL_GetTick();

	throttle = 0;

	while (true) {
		status = mpu->CheckNewData();
		// TODO read euler

		if (status == 1) {
			if (data_received)
				HAL_IWDG_Refresh(&hiwdg);
			data_received = false;


			/* Kalman begin */


			mpu->ReadAccel(&accel_data);
			mpu->ReadGyro(&gyro_data);
			mpu->ReadEuler(&euler_data);

			Calculate();

			data[0] = kalAngleX;
			data[1] = euler_data.x;
			data[2] = 0;
			/* Kalman end */



			uint8_t logData[16];
			int32_t tdata[3];
			tdata[0] = data[0] * 65536.f;
			tdata[1] = data[1] * 65536.f;
			tdata[2] = data[2] * 65536.f;

			logData[0] = (tdata[0] >> 24) & 0xFF;
			logData[1] = (tdata[0] >> 16) & 0xFF;
			logData[2] = (tdata[0] >> 8) & 0xFF;
			logData[3] = tdata[0] & 0xFF;
			logData[4] = (tdata[1] >> 24) & 0xFF;
			logData[5] = (tdata[1] >> 16) & 0xFF;
			logData[6] = (tdata[1] >> 8) & 0xFF;
			logData[7] = tdata[1] & 0xFF;
			logData[8] = (tdata[2] >> 24) & 0xFF;
			logData[9] = (tdata[2] >> 16) & 0xFF;
			logData[10] = (tdata[2] >> 8) & 0xFF;
			logData[11] = tdata[2] & 0xFF;
			logData[12] = (throttle >> 24) & 0xFF;
			logData[13] = (throttle >> 16) & 0xFF;
			logData[14] = (throttle >> 8) & 0xFF;
			logData[15] = throttle & 0xFF;

			esp->Get_Connection('4')->Connection_Send_Begin(logData, 16);
			while (esp->Get_Connection('4')->Get_State() != CONNECTION_READY && esp->Get_Connection('4')->Get_State() != CONNECTION_CLOSED) {
					esp->Get_Connection('4')->Connection_Send_Continue();
			}
		}

#ifdef LOG
		if (status == 1) {
			uint8_t logData[16];
			int32_t tdata[3];
			tdata[0] = data[0] * 65536.f;
			tdata[1] = data[1] * 65536.f;
			tdata[2] = data[2] * 65536.f;

			logData[0] = (tdata[0] >> 24) & 0xFF;
			logData[1] = (tdata[0] >> 16) & 0xFF;
			logData[2] = (tdata[0] >> 8) & 0xFF;
			logData[3] = tdata[0] & 0xFF;
			logData[4] = (tdata[1] >> 24) & 0xFF;
			logData[5] = (tdata[1] >> 16) & 0xFF;
			logData[6] = (tdata[1] >> 8) & 0xFF;
			logData[7] = tdata[1] & 0xFF;
			logData[8] = (tdata[2] >> 24) & 0xFF;
			logData[9] = (tdata[2] >> 16) & 0xFF;
			logData[10] = (tdata[2] >> 8) & 0xFF;
			logData[11] = tdata[2] & 0xFF;
			logData[12] = (throttle >> 24) & 0xFF;
			logData[13] = (throttle >> 16) & 0xFF;
			logData[14] = (throttle >> 8) & 0xFF;
			logData[15] = throttle & 0xFF;
			logger->Print(logData, 16);
		}
#endif

		if (status == 1 && throttle >= 1050) {
			pitchCorrection = PID_Pitch.get_pid(data[1], 1);
			rollCorrection = PID_Roll.get_pid(data[0], 1);
			yawCorrection = PID_Yaw.get_pid(data[2], 1);

			// not sure about yaw signs
			FL = throttle - rollCorrection + pitchCorrection /*+ yawCorrection*/; // PB2
			BL = throttle - rollCorrection - pitchCorrection /*- yawCorrection*/; // PA15
			FR = throttle + rollCorrection + pitchCorrection /*- yawCorrection*/; // PB10
			BR = throttle + rollCorrection - pitchCorrection /*+ yawCorrection*/; // PA1

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

			pwm->SetPulse(FL, 3);
			pwm->SetPulse(BL, 2);
			pwm->SetPulse(FR, 4);
			pwm->SetPulse(BR, 1);
		}
		else if (status == 2)
			LEDs::TurnOn(LEDs::Orange);

		if (throttle < 1050) {
			pwm->SetPulse(940, 4);
			pwm->SetPulse(940, 1);
			pwm->SetPulse(940, 3);
			pwm->SetPulse(940, 2);
		}

		/*if (ms5611->D1_Ready())
			ms5611->ConvertD2();
		else if (ms5611->D2_Ready()) {
			ms5611->GetData(&temp, &press);
			ms5611->ConvertD1();
		}*/

		esp->Process_Data();
		//neo->ParseData();
	}
}

void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length) {
	switch (length) {
	case 10:
		if (data[0] == 0x5D && data[9] == 0x5D) {
			data_received = true;

			throttle = data[1] << 8;
			throttle |= data[2];

			rollKp = data[3] << 8;
			rollKp |= data[4];

			pitchKp = data[5] << 8;
			pitchKp |= data[6];

			yawKp = data[7] << 8;
			yawKp |= data[8];

			PID_Roll.kP(rollKp / 100.0);
			PID_Pitch.kP(pitchKp / 100.0);
			PID_Yaw.kP(yawKp / 100.0);

			throttle += 1000;
		}
		break;
	case 8:
		if (data[0] == 0x5D && data[7] == 0x5D) {
			rollKp = data[1] << 8;
			rollKp |= data[2];

			pitchKp = data[3] << 8;
			pitchKp |= data[4];

			yawKp = data[5] << 8;
			yawKp |= data[6];

			PID_Roll.kP(rollKp / 100.0);
			PID_Pitch.kP(pitchKp / 100.0);
			PID_Yaw.kP(yawKp / 100.0);

			connected = true;
		}
		break;
	case 3:
		if (data[0] == 0x3D) {
			start = true;
		}
		break;
	}
}

void Arm_Callback() {
	esp->Process_Data();
}

void InitI2C(I2C_HandleTypeDef *I2C_Handle) {
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	if (__I2C1_IS_CLK_DISABLED())
		__I2C1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_Handle->Instance = I2C1;
	I2C_Handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C_Handle->Init.ClockSpeed = 400000;
	I2C_Handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C_Handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C_Handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C_Handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	I2C_Handle->Init.OwnAddress1 = 0;
	I2C_Handle->Init.OwnAddress2 = 0;

	__HAL_I2C_RESET_HANDLE_STATE(I2C_Handle);

	if (HAL_I2C_DeInit(I2C_Handle) != HAL_OK)
		LEDs::TurnOn(LEDs::Orange);
	if (HAL_I2C_Init(I2C_Handle) != HAL_OK)
		LEDs::TurnOn(LEDs::Orange);
}

void Calculate() {
	dt = (HAL_GetTick() - kalman_time) / 1000.0;
	kalman_time = HAL_GetTick();

	roll  = atan2(accel_data.y, accel_data.z) * 180 / PI;
	pitch = atan(-accel_data.x / sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180 / PI;

	double gyroXrate = gyro_data.x;
	double gyroYrate = gyro_data.y;

	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	}
	else
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);


	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;



}
