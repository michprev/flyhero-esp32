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

#ifdef DEBUG
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

uint16_t prev1 = 0x00;
uint16_t prev2 = 0x00;
uint16_t prev3 = 0x00;
uint16_t prev4 = 0x00;

void IPD_Callback(uint8_t *data, uint16_t length);

int main(void)
{
	HAL_Init();
#ifdef DEBUG
	initialise_monitor_handles();
#endif

	LEDs::Init();

	pwm->Init();
	pwm->Arm();

	// reset barometer
	ms5611->Init();

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

	// reset gyro
	uint8_t result;
	if (result = mpu->Init()) {
		printf("MPU error %d\n", result);
		LEDs::TurnOn(LEDs::Orange);
	}

	int32_t press, temp;
	long data[3];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	uint8_t accuracy;
	uint8_t udpD[22];
	timestamp = HAL_GetTick();

	LEDs::TurnOn(LEDs::Green);

	ms5611->ConvertD1();

	while (true) {
		mpu->CheckNewData(data, &accuracy);

		if (ms5611->D1_Ready())
			ms5611->ConvertD2();
		else if (ms5611->D2_Ready()) {
			ms5611->GetData(&temp, &press);
			ms5611->ConvertD1();
		}

		esp8266->WaitReady(0);

		if (HAL_GetTick() - timestamp >= 1000) {

			udpD[0] = (data[0] >> 24) & 0xFF;
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
			udpD[19] = press & 0xFF;

			uint32_t tmpTime = HAL_GetTick() - timestamp;

			udpD[20] = (tmpTime >> 8) & 0xFF;
			udpD[21] = tmpTime & 0xFF;

			esp8266->SendUDP(udpD, 22);

			timestamp += tmpTime;
		}
	}
}

void IPD_Callback(uint8_t *data, uint16_t length) {
	switch (length) {
	case 10:
		if (data[0] == 0x4D && data[9] == 0x4D) {
			uint16_t motor1, motor2, motor3, motor4;

			motor1 = data[1] << 8;
			motor1 |= data[2];
			motor2 = data[3] << 8;
			motor2 |= data[4];
			motor3 = data[5] << 8;
			motor3 |= data[6];
			motor4 = data[7] << 8;
			motor4 |= data[8];

			if (prev1 != motor1) {
				pwm->SetPulse(motor1 == 0xFFFF ? 950 : motor1 + 1000, 0);
				prev1 = motor1;
			}
			if (prev2 != motor2) {
				pwm->SetPulse(motor2 == 0xFFFF ? 950 : motor2 + 1000, 1);
				prev2 = motor2;
			}
			if (prev3 != motor3) {
				pwm->SetPulse(motor3 == 0xFFFF ? 950 : motor3 + 1000, 2);
				prev3 = motor3;
			}
			if (prev4 != motor4) {
				pwm->SetPulse(motor4 == 0xFFFF ? 950 : motor4 + 1000, 3);
				prev4 = motor4;
			}
		}
		break;
	case 3:
		if (data[0] == 0x3D) {

		}
		else if (data[0] == 0x2D) {

		}
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
