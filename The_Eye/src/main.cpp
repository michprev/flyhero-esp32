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
#include "Kalman.h"
#include "math.h"

using namespace flyhero;


#define PI 3.14159265358979323846

#ifdef LOG
extern "C" void initialise_monitor_handles(void);
#endif

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

ESP *esp = ESP8266::Instance();
PWM_Generator *pwm = PWM_Generator::Instance();
MPU6050 *mpu = MPU6050::Instance();
MS5611 *ms5611 = MS5611::Instance();
NEO_M8N *neo = NEO_M8N::Instance();
Logger *logger = Logger::Instance();

void Arm_Callback();
void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length);
void Calculate();

PID PID_Roll, PID_Pitch, PID_Yaw;
bool connected = false;
bool start = false;
bool data_received = false;
bool inverse_yaw = false;
uint16_t throttle = 0;
IWDG_HandleTypeDef hiwdg;

/* Testing */
/*Kalman kalmanX, kalmanY;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t kalman_time;
double dt;
MPU6050::Sensor_Data accel_data;
MPU6050::Sensor_Data gyro_data;*/
MPU6050::Sensor_Data euler_data;
//double roll, pitch;

/* End Testing */

MPU6050::Raw_Data gyroData, accelData;

/*uint32_t test;
uint32_t aa[1000];
uint32_t ppos = 0;

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART5) {
		aa[ppos] = Timer::Get_Tick_Count() - test;
		ppos++;

		if (ppos == 1000) {
			printf("a");
		}
	}
}*/

int main(void)
{
	HAL_Init();
#ifdef LOG
	initialise_monitor_handles();
#endif

	uint8_t status;
	uint32_t timestamp;
	float data[3];
	long FL, BL, FR, BR;
	FL = BL = FR = BR = 0;
	long pitchCorrection, rollCorrection, yawCorrection;
	double reference_yaw;

	PID_Roll.imax(50);
	PID_Pitch.imax(50);
	PID_Yaw.imax(50);

	LEDs::Init();

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 480;
	// timeout after 2 s

	logger->Init();
	esp->Init(&IPD_Callback);

	timestamp = HAL_GetTick();
	// not yet implemented
	//Timer::Start_Task([]{ LEDs::Toggle(LEDs::Green); }, 750);

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

	// reset gyro
	if (mpu->Init(false)) {
		LEDs::TurnOn(LEDs::Yellow);
		while (true);
	}

	pwm->SetPulse(1100, 1);
	pwm->SetPulse(1100, 2);
	pwm->SetPulse(1100, 3);
	pwm->SetPulse(1100, 4);

	Timer::Delay_ms(250);

	pwm->SetPulse(940, 4);
	pwm->SetPulse(940, 1);
	pwm->SetPulse(940, 3);
	pwm->SetPulse(940, 2);

	while (!start) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp->Process_Data();
	}

	/*while (!mpu->ReadAccel(&accel_data));
	roll  = atan2(accel_data.y, accel_data.z) * 180 / PI;
	pitch = atan(-accel_data.x / sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180 / PI;

	kalmanX.setAngle(roll);
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compAngleX = roll;
	compAngleY = pitch;

	kalman_time = HAL_GetTick();*/


	LEDs::TurnOn(LEDs::Green);

#ifndef LOG
	HAL_IWDG_Init(&hiwdg);
#endif

	throttle = 0;
	mpu->ready = true;

	while (true) {
		if (mpu->Data_Ready()) {
			if (mpu->Start_Read_Raw() != HAL_OK) {
				LEDs::TurnOn(LEDs::Orange);
			}
		}

		if (mpu->Data_Read()) {
			if (data_received)
				HAL_IWDG_Refresh(&hiwdg);
			data_received = false;

			mpu->Complete_Read_Raw(&gyroData, &accelData);

			/*uint8_t tmp[15];
			tmp[0] = accelData.x >> 8;
			tmp[1] = accelData.x & 0xFF;
			tmp[2] = accelData.y >> 8;
			tmp[3] = accelData.y & 0xFF;
			tmp[4] = accelData.z >> 8;
			tmp[5] = accelData.z & 0xFF;
			tmp[6] = gyroData.x >> 8;
			tmp[7] = gyroData.x & 0xFF;
			tmp[8] = gyroData.y >> 8;
			tmp[9] = gyroData.y & 0xFF;
			tmp[10] = gyroData.z >> 8;
			tmp[11] = gyroData.z & 0xFF;
			tmp[12] = throttle >> 8;
			tmp[13] = throttle & 0xFF;
			tmp[14] = 0;

			for (uint8_t i = 0; i <= 13; i++)
				tmp[14] ^= tmp[i];

			logger->Print(tmp, 15);*/


			data[0] = 0;
			data[1] = 0;
			data[2] = 0;

			if (throttle >= 1050) {
				pitchCorrection = PID_Pitch.get_pid(data[1], 1);
				rollCorrection = PID_Roll.get_pid(data[0], 1);
				yawCorrection = PID_Yaw.get_pid(data[2] - reference_yaw, 1);

				// not sure about yaw signs
				if (!inverse_yaw) {
					FL = throttle + rollCorrection + pitchCorrection + yawCorrection; // PB2
					BL = throttle + rollCorrection - pitchCorrection - yawCorrection; // PA15
					FR = throttle - rollCorrection + pitchCorrection - yawCorrection; // PB10
					BR = throttle - rollCorrection - pitchCorrection + yawCorrection; // PA1
				}
				else {
					FL = throttle + rollCorrection + pitchCorrection - yawCorrection; // PB2
					BL = throttle + rollCorrection - pitchCorrection + yawCorrection; // PA15
					FR = throttle - rollCorrection + pitchCorrection + yawCorrection; // PB10
					BR = throttle - rollCorrection - pitchCorrection - yawCorrection; // PA1
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

				pwm->SetPulse(FL, 3);
				pwm->SetPulse(BL, 2);
				pwm->SetPulse(FR, 4);
				pwm->SetPulse(BR, 1);
			}
			else {
				pwm->SetPulse(940, 4);
				pwm->SetPulse(940, 1);
				pwm->SetPulse(940, 3);
				pwm->SetPulse(940, 2);
				reference_yaw = euler_data.z;
			}
		}

		esp->Process_Data();
	}
}

void IPD_Callback(uint8_t link_ID, uint8_t *data, uint16_t length) {
	switch (length) {
	case 22:
		// around 100 us
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
			PID_Roll.kP(roll_kP * 0.01);
			PID_Roll.kI(roll_kI * 0.01);
			PID_Roll.kD(roll_kD * 0.01);

			pitch_kP = data[9] << 8;
			pitch_kP |= data[10];

			pitch_kI = data[11] << 8;
			pitch_kI |= data[12];

			pitch_kD = data[13] << 8;
			pitch_kD |= data[14];

			PID_Pitch.kP(pitch_kP * 0.01);
			PID_Pitch.kI(pitch_kI * 0.01);
			PID_Pitch.kD(pitch_kD * 0.01);

			yaw_kP = data[15] << 8;
			yaw_kP |= data[16];

			yaw_kI = data[17] << 8;
			yaw_kI |= data[18];

			yaw_kD = data[19] << 8;
			yaw_kD |= data[20];

			PID_Yaw.kP(yaw_kP / 100.0);
			PID_Yaw.kI(yaw_kI / 100.0);
			PID_Yaw.kD(yaw_kD / 100.0);


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
		}
		break;
	}
}

void Arm_Callback() {
	esp->Process_Data();
	//HAL_IWDG_Refresh(&hiwdg);
}

/*void Calculate() {
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



}*/
