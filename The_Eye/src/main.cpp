/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "PWM_Generator.h"
#include "ESP8266_UDP.h"
#include "MS5611.h"
#include "MPU9250.h"
#include "LEDs.h"
#include "NEO_M8N.h"
#include "PID.h"

#ifdef LOG
extern "C" void initialise_monitor_handles(void);
#endif

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

ESP8266_UDP *esp8266 = ESP8266_UDP::Instance();
PWM_Generator *pwm = PWM_Generator::Instance();
MPU9250 *mpu = MPU9250::Instance();
MS5611 *ms5611 = MS5611::Instance();
NEO_M8N *neo = NEO_M8N::Instance();

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	mpu->dataReady = true;
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&esp8266->hdma_usart3_rx);
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&neo->hdma_usart1_rx);
}

void InitI2C(I2C_HandleTypeDef*);

void IPD_Callback(uint8_t *data, uint16_t length);


uint16_t throttle = 0;


int main(void)
{
	HAL_Init();
#ifdef LOG
	initialise_monitor_handles();
#endif

	int32_t press, temp;
	long data[3];
	uint8_t accuracy;
	uint8_t udpD[22];

	long FL, BL, FR, BR;
	long pitchCorrection, rollCorrection;

	PID PID_Pitch;
	PID_Pitch.kP(0.7);
	//PID_Pitch.kI(1);
	PID_Pitch.imax(50);

	PID PID_Roll;
	PID_Roll.kP(0.7);
	//PID_Roll.kI(1);
	PID_Roll.imax(50);

	PID PID_Yaw;
	PID_Yaw.kP(2.5);
	PID_Yaw.imax(50);

	LEDs::Init();


	I2C_HandleTypeDef hI2C_Handle;

	InitI2C(&hI2C_Handle);

	// reset ESP8266
	esp8266->Reset();

	uint32_t timestamp = HAL_GetTick();
	while (!esp8266->ready) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Yellow);

			timestamp = HAL_GetTick();
		}
		esp8266->WaitReady(0);
	}
	LEDs::TurnOff(LEDs::Yellow);

	esp8266->IPD_Callback = &IPD_Callback;
	esp8266->output = true;
	esp8266->Init();

	timestamp = HAL_GetTick();
	while (!esp8266->handshaken) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp8266->WaitReady(0);
	}
	LEDs::TurnOff(LEDs::Green);

	//pwm->Init();
	//pwm->Arm();

	// reset gyro
	if (mpu->Init(&hI2C_Handle)) {
		LEDs::TurnOn(LEDs::Yellow);
	}

	// reset barometer
	if (ms5611->Init(&hI2C_Handle) != HAL_OK) {
		LEDs::TurnOn(LEDs::Orange);
	}

	//neo->Init();

	ms5611->ConvertD1();

	// everything OK
	LEDs::TurnOn(LEDs::Green);

	timestamp = HAL_GetTick();

	uint8_t status;

	while (true) {
		status = mpu->CheckNewData(data, &accuracy);

		if (status == 1 && throttle >= 1100) {
			pitchCorrection = PID_Pitch.get_pid(data[1], 1);
			rollCorrection = PID_Roll.get_pid(data[0], 1);

			FL = throttle - rollCorrection + pitchCorrection; // PB2
			BL = throttle - rollCorrection - pitchCorrection; // PA15
			FR = throttle + rollCorrection + pitchCorrection; // PB10
			BR = throttle + rollCorrection - pitchCorrection; // PA1

			pwm->SetPulse(FL, 4);
			pwm->SetPulse(BL, 1);
			pwm->SetPulse(FR, 3);
			pwm->SetPulse(BR, 2);
		}
		else if (status == 2)
			LEDs::TurnOn(LEDs::Orange);

		if (ms5611->D1_Ready())
			ms5611->ConvertD2();
		else if (ms5611->D2_Ready()) {
			ms5611->GetData(&temp, &press);
			ms5611->ConvertD1();
		}

		esp8266->WaitReady(0);
		//neo->ParseData();

		if (HAL_GetTick() - timestamp >= 1000) {
			/*udpD[0] = (data[0] >> 24) & 0xFF;
			udpD[1] = (data[0] >> 16) & 0xFF;
			udpD[2] = (data[0] >> 8) & 0xFF;
			udpD[3] = data[0] & 0xFF;
			udpD[4] = (data[1] >> 24) & 0xFF;
			udpD[5] = (data[1] >> 16) & 0xFF;
			udpD[6] = (data[1] >> 8) & 0xFF;
			udpD[7] = data[1] & 0xFF;
			udpD[8] = (data[2] >> 24) & 0xFF;
			udpD[9] = (data[2] >> 16) & 0xFF;
			udpD[10] = (data[2] >> 8) & 0xFF;
			udpD[11] = data[2] & 0xFF;
			udpD[12] = (temp >> 24) & 0xFF;
			udpD[13] = (temp >> 16) & 0xFF;
			udpD[14] = (temp >> 8) & 0xFF;
			udpD[15] = temp & 0xFF;
			udpD[16] = (press >> 24) & 0xFF;
			udpD[17] = (press >> 16) & 0xFF;
			udpD[18] = (press >> 8) & 0xFF;
			udpD[19] = press & 0xFF;*/

			udpD[0] = (FL >> 8) & 0xFF;
			udpD[1] = FL & 0xFF;
			udpD[2] = (BL >> 8) & 0xFF;
			udpD[3] = BL & 0xFF;
			udpD[4] = (FR >> 8) & 0xFF;
			udpD[5] = FR & 0xFF;
			udpD[6] = (BR >> 8) & 0xFF;
			udpD[7] = BR & 0xFF;

			uint32_t tmpTime = HAL_GetTick() - timestamp;

			udpD[8] = (tmpTime >> 8) & 0xFF;
			udpD[9] = tmpTime & 0xFF;

			esp8266->SendUDP(udpD, 10);

			timestamp += tmpTime;
		}
	}
}

void IPD_Callback(uint8_t *data, uint16_t length) {
	switch (length) {
	case 10:
		if (data[0] == 0x4D && data[9] == 0x4D) {

		}
		break;
	case 3:
		if (data[0] == 0x3D) {

		}
		else if (data[0] == 0x2D) {

		}
		// run self-test
		else if (data[0] == 0x1D) {
			LEDs::TurnOff(LEDs::Green);

			if (mpu->SelfTest())
				LEDs::TurnOn(LEDs::Green);
			else
				LEDs::TurnOn(LEDs::Orange);
		}
		break;
	}
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

	HAL_I2C_Init(I2C_Handle);
}
