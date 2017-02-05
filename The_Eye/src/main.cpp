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

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&esp8266->hdma_usart3_tx);
}

extern "C" void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&esp8266->huart);
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&neo->hdma_usart1_rx);
}

void InitI2C(I2C_HandleTypeDef*);
void Arm_Callback();
void IPD_Callback(uint8_t *data, uint16_t length);

bool IWDG_Started = false;
uint16_t rollKp, pitchKp;
bool connected = false;
bool start = false;
uint16_t throttle = 0;
IWDG_HandleTypeDef hiwdg;

uint8_t FL_offset = 0;
uint8_t BL_offset = 0;
uint8_t FR_offset = 0;
uint8_t BR_offset = 0;

int main(void)
{
	HAL_Init();
#ifdef LOG
	initialise_monitor_handles();
#endif
	uint8_t status;
	//int32_t press, temp;
	float data[3];
	uint8_t accuracy;
	uint8_t udpD[22];
	uint8_t udpLen;
	long FL, BL, FR, BR;
	FL = BL = FR = BR = 0;
	long pitchCorrection, rollCorrection;

	LEDs::Init();

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 480;
	// timeout after 2 s
	HAL_IWDG_Init(&hiwdg);

	I2C_HandleTypeDef hI2C_Handle;
	InitI2C(&hI2C_Handle);

	// reset ESP8266
	esp8266->Reset();

	uint32_t timestamp = HAL_GetTick();
	while (!esp8266->Ready) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Yellow);

			timestamp = HAL_GetTick();
		}
		esp8266->ProcessData();
	}
	LEDs::TurnOff(LEDs::Yellow);

	esp8266->IPD_Callback = &IPD_Callback;
	esp8266->Init();

	timestamp = HAL_GetTick();
	while (!connected) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		esp8266->ProcessData();
	}
	LEDs::TurnOff(LEDs::Green);

	PID PID_Roll;
	PID_Roll.kP(rollKp / 100.0);
	//PID_Roll.kI(1);
	PID_Roll.imax(50);

	PID PID_Pitch;
	PID_Pitch.kP(pitchKp / 100.0);
	//PID_Pitch.kI(1);
	PID_Pitch.imax(50);

	/*PID PID_Yaw;
	PID_Yaw.kP(2.5);
	PID_Yaw.imax(50);*/

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

	while (!start) {
		if (HAL_GetTick() - timestamp >= 750) {
			LEDs::Toggle(LEDs::Green);

			timestamp = HAL_GetTick();
		}
		mpu->CheckNewData(data, &accuracy);
		esp8266->ProcessData();
	}
	LEDs::TurnOff(LEDs::Green);


	LEDs::TurnOn(LEDs::Green);

	//ms5611->ConvertD1();
	HAL_IWDG_Start(&hiwdg);
	IWDG_Started = true;

	timestamp = HAL_GetTick();

	uint32_t wifi = HAL_GetTick();

	while (true) {
		status = mpu->CheckNewData(data, &accuracy);

		if (status == 1 && throttle >= 1050) {
			pitchCorrection = PID_Pitch.get_pid(data[1], 1);
			rollCorrection = PID_Roll.get_pid(data[0], 1);

			FL = throttle - rollCorrection + pitchCorrection + FL_offset; // PB2
			BL = throttle - rollCorrection - pitchCorrection + BL_offset; // PA15
			FR = throttle + rollCorrection + pitchCorrection + FR_offset; // PB10
			BR = throttle + rollCorrection - pitchCorrection + BR_offset; // PA1

			pwm->SetPulse(FL, 4);
			pwm->SetPulse(BL, 1);
			pwm->SetPulse(FR, 3);
			pwm->SetPulse(BR, 2);
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

		if (esp8266->State != ESP_READY && HAL_GetTick() - wifi >= 100)
			esp8266->State = ESP_READY;

		if (esp8266->State == ESP_READY && HAL_GetTick() - wifi >= 100) {
			long rollD, pitchD;
			rollD = data[0] * 65536;
			pitchD = data[1] * 65536;

			uint8_t tmpD[8];

			tmpD[0] = (rollD >> 24) & 0xFF;
			tmpD[1] = (rollD >> 16) & 0xFF;
			tmpD[2] = (rollD >> 8) & 0xFF;
			tmpD[3] = rollD & 0xFF;
			tmpD[4] = (pitchD >> 24) & 0xFF;
			tmpD[5] = (pitchD >> 16) & 0xFF;
			tmpD[6] = (pitchD >> 8) & 0xFF;
			tmpD[7] = pitchD & 0xFF;

			uint8_t pos = 0;

			for (uint8_t i = 0; i < 7; i++) {
				if (tmpD[i] == '\\' && tmpD[i + 1] == '\0') {
					udpD[pos] = '\\';
					udpD[pos + 1] = '\\';
					udpD[pos + 2] = '\\';
					i++;
					pos += 3;
				}
				else {
					udpD[pos] = tmpD[i];
					pos++;
				}
			}

			if (tmpD[6] != '\\' || tmpD[7] != '\0') {
				udpD[pos] = tmpD[7];
			}

			udpLen = pos + 1;

			esp8266->SendUDP_Header(udpLen);
			wifi = HAL_GetTick();
		}

		esp8266->ProcessData();

		if (esp8266->State == ESP_AWAITING_BODY) {
			esp8266->SendUDP(udpD, udpLen);
			esp8266->ProcessData();
		}

		//neo->ParseData();

		/*if (HAL_GetTick() - timestamp >= 500) {
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
		}*/
	}
}

void IPD_Callback(uint8_t *data, uint16_t length) {
	switch (length) {
	case 8:
		if (data[0] == 0x5D && data[7] == 0x5D) {
			if (IWDG_Started)
				HAL_IWDG_Refresh(&hiwdg);

			throttle = data[1] << 8;
			throttle |= data[2];

			if (data[3] == 0xFF)
				FL_offset = 0;
			else
				FL_offset = data[3];

			if (data[4] == 0xFF)
				BL_offset = 0;
			else
				BL_offset = data[4];

			if (data[5] == 0xFF)
				FR_offset = 0;
			else
				FR_offset = data[5];

			if (data[6] == 0xFF)
				BR_offset = 0;
			else
				BR_offset = data[6];

			throttle += 1000;
		}
		break;
	case 6:
		if (data[0] == 0x5D && data[5] == 0x5D) {
			rollKp = data[1] << 8;
			rollKp |= data[2];

			pitchKp = data[3] << 8;
			pitchKp |= data[4];

			connected = true;
		}
		break;
	case 3:
		if (data[0] == 0x3D) {
			start = true;
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

void Arm_Callback() {
	esp8266->ProcessData();
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
